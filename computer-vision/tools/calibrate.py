#!/usr/bin/env python3
"""
Computes the calibration matrix of a camera given a number of sample images.
"""
import argparse
import numpy as np
import cv2


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action="store_true", help="Show detected chessboard.")
    parser.add_argument(
        "-c", "--config", help="Output config file path.", default="config.yaml"
    )
    parser.add_argument("images", nargs="+", help="Images to process, in jpg format.")

    return parser.parse_args()


def main():
    args = parse_args()

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((9 * 6, 3), np.float32)
    objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    gray = []
    for fname in args.images:
        img = cv2.imread(fname, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (6, 9), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            if args.show:
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (6, 9), corners2, ret)
                cv2.imshow("img", img)
                cv2.waitKey(1000)

    cv2.destroyAllWindows()

    shape = gray.shape[::-1]
    err, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, shape, None, None
    )

    print("shape\n", shape)
    print("camera_matrix\n", mtx)
    print("distortion_coefficients\n", dist)

    fs = cv2.FileStorage(args.config, cv2.FileStorage_WRITE)
    fs.write("image_width", shape[0])
    fs.write("image_height", shape[1])
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.release()


if __name__ == "__main__":
    main()
