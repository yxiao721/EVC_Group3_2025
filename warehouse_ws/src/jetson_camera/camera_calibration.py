#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import glob

# === Define chessboard pattern parameters ===
# Number of inner corners per chessboard row and column
# pattern_size = (5, 7)  # 7 corners per row, 5 per column
pattern_size = (6, 9)  # Our new chessboard on a laptop - 9 corners per row, 6 per column
square_size = 21  # Length of each square in mm

# Prepare object points like: (0,0,0), (31,0,0), (62,0,0) ... assuming chessboard lies on the Z=0 plane
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

# Lists to store points for all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# === Load images from calibration_images/ folder ===
images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Try to find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        # Refine corner positions
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners2)

        # Optional: draw and show corners
        cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
        cv2.imshow('Detected Corners', img)
        key = cv2.waitKey(0)
        if key == 27:  
            break


cv2.destroyAllWindows()

# === Calibrate camera ===
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# === Save result ===
np.savez('calibration_result.npz', mtx=mtx, dist=dist)

# === Print for confirmation ===
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)
