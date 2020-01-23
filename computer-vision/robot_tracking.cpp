#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

#include "protobuf/protocol.pb.h"
#include "protobuf/computer_vision.pb.h"

#define ARUCO_CENTER 42
#define ARUCO_BLUE_FIRST 1
#define ARUCO_BLUE_LAST 5
#define ARUCO_YELLOW_FIRST 6
#define ARUCO_YELLOW_LAST 10

struct RobotTrackingSettings {
    int image_width;
    int image_height;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;

    void load(const std::string& file)
    {
        int flags = cv::FileStorage::READ; // cv::FileStorage::FORMAT_YAML
        cv::FileStorage fs = cv::FileStorage(file, flags);

        cv::read(fs["image_width"], image_width, -1);
        cv::read(fs["image_height"], image_height, -1);
        cv::read(fs["camera_matrix"], camera_matrix);
        cv::read(fs["distortion_coefficients"], distortion_coefficients);
    }
};

void processImage(cv::Mat& image, RobotTrackingSettings& settings, bool display = false)
{
    float markerLength = 0.1;
    const int dictionaryId = 1; // DICT_4X4_100=1

    const cv::Mat& M = settings.camera_matrix;
    const cv::Mat& D = settings.distortion_coefficients;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<cv::Vec3d> rvecs, tvecs;

    // detect markers and estimate pose
    cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    if (ids.size() > 0) {
        cv::aruco::estimatePoseSingleMarkers(corners, markerLength, M, D, rvecs,
                                             tvecs);
    }

    for (unsigned int i = 0; i < ids.size(); i++) {
        std::cout << ids[i] << std::endl;
        std::cout << rvecs[i] << std::endl;
        std::cout << tvecs[i] << std::endl;
    }

    if (display) {
        cv::Mat imageCopy;
        // draw results
        image.copyTo(imageCopy);

        std::cout << "found " << ids.size() << " markers." << std::endl;
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        for (unsigned int i = 0; i < ids.size(); i++) {
            cv::aruco::drawAxis(imageCopy, M, D, rvecs[i], tvecs[i],
                                markerLength * 0.5f);
        }

        // cv::imwrite("out.jpg", imageCopy);
        cv::imshow("out", imageCopy);
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("usage: %s config.yaml image.jpg\n", argv[0]);
        return -1;
    }

    RobotTrackingSettings settings;
    settings.load(argv[1]);

    std::cout << settings.image_width << std::endl;
    std::cout << settings.image_height << std::endl;
    std::cout << settings.camera_matrix << std::endl;
    std::cout << settings.distortion_coefficients << std::endl;

    ComputerVisionResult result;
    result.set_weather_vane_orientation(ComputerVisionResult::NORTH);
    std::cout << result.DebugString() << std::endl;

    cv::Mat image;
    if (argc == 2) {
        const int camId = 0;
        cv::VideoCapture inputVideo;
        inputVideo.open(camId);
        while (inputVideo.grab()) {
            inputVideo.retrieve(image);
            processImage(image, settings, true);
            char key = (char)cv::waitKey(10);
            if (key == 27) {
                break;
            }
        }
    } else {
        image = cv::imread(argv[2], cv::IMREAD_COLOR);
        processImage(image, settings, true);
        while (true) {
            char key = (char)cv::waitKey(10);
            if (key == 27) {
                break;
            }
        }
    }

    return 0;
}
