# CVRA 2019 Computer vision


Requirements:

- Detect weathervane orientation using its Aruco marker (6cm 4x4 ArUco Nb 17 at [1500,0])
- Detect the random reefs configuration
- Send this to the master firmware

## How to run

The project requires OpenCV and Protobuf.
It is packaged as a cmake project, so to run it, run the following:

```sh
cd computer-vision
mkdir build
cd build
cmake ..

# On subsequent builds, only the following are required
make
./robot_tracking ../config/iphone.yaml
```

## Camera calibration

Follow the OpenCV tutorial: [Camera calibration With OpenCV](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html).
You may also read [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html).

Take some picture with the script `tools/grap_image.py` and then run `tools/calibrate.py` to get the camera matrix and distortion coefficient vector.

Note: Fisheye camera need different procedure using [fisheye::calibrate](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=calibratecamera#fisheye-calibrate)

## ArUco tracking

Start with the [OpenCV Tutorial: ArUco detection](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html).
