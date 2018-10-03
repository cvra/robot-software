#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>


namespace seeker {
struct RGB {
    uint8_t r, g, b;
    RGB(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {};
};
std::vector<RGB> color_palette(size_t number_of_colors);
}  // seeker


int main (int argc, char** argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<sensor_msgs::PointCloud2>("objects", 1);

    auto on_new_point_cloud = boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>(
        [&pub](const sensor_msgs::PointCloud2ConstPtr& msg) {
            ROS_INFO("I can see clearly now");

            // From ROS to PCL
            pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
            pcl_conversions::toPCL(*msg, *cloud_blob);

            ///////////////////////////////////////

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
            ROS_DEBUG_STREAM("PointCloud before filtering has: " << cloud->points.size() << " data points.");

            // Crop to bounding box
            pcl::CropBox<pcl::PointXYZ> boxFilter;
            boxFilter.setMin(Eigen::Vector4f(-2.0, -1.0,  0.0, 1.0));
            boxFilter.setMax(Eigen::Vector4f( 2.0,  2.0,  2.0, 1.0));
            boxFilter.setInputCloud(cloud);
            boxFilter.filter(*cloud);

            // Downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            vg.setInputCloud(cloud);
            vg.setLeafSize(0.01f, 0.01f, 0.01f);
            vg.filter(*cloud_filtered);
            ROS_DEBUG_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points.");

            if (cloud_filtered->points.size() == 0) {
                ROS_DEBUG_STREAM("PointCloud after filtering is empty, waiting for next one.");
                return;
            }

            // Setup segmentation for the planar model and set all the parameters
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(100);
            seg.setDistanceThreshold(0.02);

            int i=0, nr_points = (int) cloud_filtered->points.size();
            while (cloud_filtered->points.size() > 0.3 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_filtered);
                seg.segment(*inliers, *coefficients);
                if (inliers->indices.size() == 0)
                {
                    ROS_DEBUG_STREAM("Could not estimate a planar model for the given dataset.");
                    break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(false);

                // Get the points associated with the planar surface
                extract.filter(*cloud_plane);
                ROS_DEBUG_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size() << " data points.");

                // Remove the planar inliers, extract the rest
                extract.setNegative(true);
                extract.filter(*cloud_f);
                *cloud_filtered = *cloud_f;
            }

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.01); // 2cm
            ec.setMinClusterSize(30);
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_filtered);
            ec.extract(cluster_indices);

            int j = 0;
            auto palette = seeker::color_palette(cluster_indices.size());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (const auto& cluster_index : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZRGB>);

                for (const auto& point_index : cluster_index.indices) {
                    pcl::PointXYZRGB point(palette[j].r, palette[j].g, palette[j].b);
                    point.x = cloud_filtered->points[point_index].x;
                    point.y = cloud_filtered->points[point_index].y;
                    point.z = cloud_filtered->points[point_index].z;
                    cloud_object->points.push_back(point);
                }
                ROS_DEBUG_STREAM("Object " << j << " clustered with: " << cloud_object->points.size() << " data points.");

                pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
                feature_extractor.setInputCloud(cloud_object);
                feature_extractor.compute();

                pcl::PointXYZRGB min_point_OBB, max_point_OBB;
                pcl::PointXYZRGB position_OBB;
                Eigen::Matrix3f rotational_matrix_OBB;
                feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

                if (cluster_index.indices.size() < 100) {
                    ROS_DEBUG_STREAM("Found cluster with too few points, skipping");
                    continue;
                }

                float obb_x_size = max_point_OBB.x - min_point_OBB.x;
                float obb_y_size = max_point_OBB.y - min_point_OBB.y;
                float obb_z_size = max_point_OBB.z - min_point_OBB.z;

                if (obb_x_size > 0.05 && obb_y_size > 0.05 && obb_z_size > 0.05
                    && obb_x_size < 0.15 && obb_y_size < 0.15 && obb_z_size < 0.15) {
                    for (const auto& point : cloud_object->points) {
                        cloud_cluster->points.push_back(point);
                    }
                    ROS_INFO_STREAM("Valid object identified");
                } else {
                    ROS_DEBUG_STREAM("Ignoring clustered object too small");
                    continue;
                }

                j++;
            }
            // Publish clusters
            cloud_cluster->header.frame_id = msg->header.frame_id;
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            pcl_conversions::toPCL(ros::Time::now(), cloud_cluster->header.stamp);
            pub.publish(cloud_cluster);
        }
    );
    ros::Subscriber sub = node.subscribe("/camera/depth_registered/points", 1, on_new_point_cloud);

    while (ros::ok()) {
        ros::spin();
    }
}

namespace seeker {
std::vector<RGB> color_palette(size_t number_of_colors)
{
    std::vector<RGB> colors;
    colors.reserve(number_of_colors);

    size_t grid_size = std::ceil(std::cbrt(number_of_colors));
    for (size_t i = 0; i < grid_size; i++) {
        for (size_t j = 0; j < grid_size; j++) {
            for (size_t k = 0; k < grid_size; k++) {
                colors.emplace_back(255.f * i / grid_size, 255.f * j /grid_size, 255.f * k / grid_size);
            }
        }
    }

    return colors;
}
}  // seeker
