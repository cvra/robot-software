#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

namespace seeker {
}  // seeker

int main (int argc, char** argv)
{
    ros::init (argc, argv, "segmentation");
    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<sensor_msgs::PointCloud2>("plane", 1);

    auto on_new_point_cloud = boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>(
        [&pub](const sensor_msgs::PointCloud2ConstPtr& msg) {
            ROS_INFO("I've seen your cloud!");

            // From ROS to PCL
            pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
            pcl_conversions::toPCL(*msg, *cloud_blob);

            ///////////////////////////////////////

            pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

            ROS_INFO_STREAM("PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points.");

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud_blob);
            sor.setLeafSize(0.01, 0.01, 0.01);
            sor.filter(*cloud_filtered_blob);

            // Convert to the templated PointCloud
            pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

            ROS_INFO_STREAM("PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points.");

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.01);

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                ROS_INFO_STREAM("Could not estimate a planar model for the given dataset.");
                return;
            }

            // Extract the inliers
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_p);

            ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points.");

            extract.setNegative(true);
            extract.filter(*cloud_f);
            cloud_filtered.swap(cloud_f);

            // Publish results
            cloud_p->header.frame_id = msg->header.frame_id;
            pcl_conversions::toPCL(ros::Time::now(), cloud_p->header.stamp);
            pub.publish(cloud_p);
        }
    );
    ros::Subscriber sub = node.subscribe("/camera/depth_registered/points", 1, on_new_point_cloud);

    while (ros::ok()) {
        ros::spin();
    }
}

namespace seeker {
}  // seeker
