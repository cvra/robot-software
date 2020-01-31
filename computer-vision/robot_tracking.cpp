#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>

#include "protobuf/protocol.pb.h"
#include "protobuf/computer_vision.pb.h"
#include "error/error.h"
#include "logging.h"
#include "udp_protocol.h"

#define ARUCO_CENTER 42
#define ARUCO_WEATHERVANE 17

struct RobotTrackingSettings {
    int image_width;
    int image_height;
    std::string destination_ip;
    int destination_port;
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

        destination_ip = (std::string)fs["destination_ip"];
        destination_port = (int)fs["destination_port"];
    }
};

enum WeathiervaneDirection {
    NORTH = 0,
    SOUTH = 1
};

WeathiervaneDirection weathervane_direction(cv::Vec3d rvec, cv::Vec3d tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Vec3d e2 = {0, 1, 0};
    cv::Vec3d v = cv::Mat(R * e2);

    // check if facing down
    if (v[1] > 0) {
        return SOUTH;
    } else {
        return NORTH;
    }
}

ComputerVisionResult processImage(cv::Mat& image, RobotTrackingSettings& settings, bool display = false)
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

    int dir = -1;
    for (size_t i = 0; i < ids.size(); i++) {
        if (ids[i] == ARUCO_WEATHERVANE) {
            dir = weathervane_direction(rvecs[i], tvecs[i]);
            break;
        }
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

    ComputerVisionResult result;
    switch (dir) {
        case NORTH:
            result.set_weather_vane_orientation(ComputerVisionResult::NORTH);
            break;
        case SOUTH:
            result.set_weather_vane_orientation(ComputerVisionResult::SOUTH);
            break;
    }

    return result;
}

int main(int argc, char* argv[])
{
    logging_init();
    if (argc < 2) {
        ERROR("usage: %s config.yaml image.jpg\n", argv[0]);
        return EXIT_FAILURE;
    }

    RobotTrackingSettings settings;
    settings.load(argv[1]);

    std::cout << settings.image_width << std::endl;
    std::cout << settings.image_height << std::endl;
    std::cout << settings.camera_matrix << std::endl;
    std::cout << settings.distortion_coefficients << std::endl;

    cv::Mat image;
    if (argc == 2) {
        const int camId = 0;
        cv::VideoCapture inputVideo;
        inputVideo.open(camId);
        while (inputVideo.grab()) {
            inputVideo.retrieve(image);
            auto result = processImage(image, settings, true);
            udp_send_result(settings.destination_ip, settings.destination_port, result);
            char key = (char)cv::waitKey(10);
            if (key == 27) {
                break;
            }
        }
    } else {
        image = cv::imread(argv[2], cv::IMREAD_COLOR);
        auto result = processImage(image, settings, true);
        udp_send_result(settings.destination_ip, settings.destination_port, result);
        while (true) {
            char key = (char)cv::waitKey(10);
            if (key == 27) {
                break;
            }
        }
    }

    return 0;
}
