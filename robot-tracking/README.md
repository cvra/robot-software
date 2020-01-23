# ArUco Robot Tracking


Requirements:

- detect 4 robot markers (7cm 4x4 ArUco, blue: 1-5, yellow: 6-10)
- detect 1 central marker for position reference (10cm 4x4 ArUco Nb 42, centred at coordinates [1500,1250])
- optionally: detect weathervane marker for "to determine wind direction" (6cm 4x4 ArUco Nb 17 at [1500,0])
- Transform robot marker position to table reference frame.
- Send position to both robots via radio link (UWB or WiFI)
- outlier rejection (e.g. ignore markers outside of game area, ignore all markers that do not belong to the game)

The tracking system should communicate only the marker label and position in the table coordinate frame. The ArUco label assignment can happen on robot as the robots know their own position and color at start of the game.

## Camera calibration

Follow the OpenCV tutorial: [Camera calibration With OpenCV](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html).
You may also read [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html).

Take some picture with the script `tools/grap_image.py` and then run `tools/calibrate.py` to get the camera matrix and distortion coefficient vector.

Note: Fisheye camera need different procedure using [fisheye::calibrate](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=calibratecamera#fisheye-calibrate)

## ArUco tracking

Start with the [OpenCV Tutorial: ArUco detection](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html).
