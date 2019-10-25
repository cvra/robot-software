#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    const int camId = 0;
    int waitTime = 10;
    float markerLength = 0.1;
    const int dictionaryId = 1; // DICT_4X4_100=1

    // TODO: load camera parameters from config file
    cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << 1.05127541e+03, 0.00000000e+00, 6.41754791e+02, 0.00000000e+00, 1.05157141e+03, 3.36663652e+02, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.11103421, -0.23824016, -0.00629281, 0.00078222, 0.16402498);

    cv::VideoCapture inputVideo;
    inputVideo.open(camId);

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    while (inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        std::vector<cv::Vec3d> rvecs, tvecs;

        // detect markers and estimate pose
        cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if (ids.size() > 0)
            cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                                 tvecs);

        // draw results
        image.copyTo(imageCopy);

        std::cout << "found " << ids.size() << " markers." << std::endl;
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            for (unsigned int i = 0; i < ids.size(); i++) {
                std::cout << i << std::endl;
                std::cout << rvecs[i] << std::endl;
                std::cout << tvecs[i] << std::endl;
                cv::aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    markerLength * 0.5f);
            }
        }

        if (rejected.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, rejected, cv::noArray(), cv::Scalar(100, 0, 255));
        }

        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(waitTime);
        if (key == 27)
            break;
    }

    return 0;
}
