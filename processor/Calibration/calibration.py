'''
@file calibration.py
@author Levi Smith
@brief Script for calibrating the camera
'''

import numpy as np
import cv2
import glob

# Define checkerboard dimensions
CHECKERBOARD = (10,7)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.001)

objpoints, imgpoints = [], []

# 3D points real world coordinates
objp3d = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp3d[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

images = glob.glob('./images/*.jpg')

image_shape = None
dimension = None
for filename in images:
	image = cv2.imread(filename)
	dimension = image.shape
	image_shape = image.shape[:2][::-1]
	grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# find chessboard corners, if desired number of corners are found in image ret = true
	ret, corners = cv2.findChessboardCorners(
		grayColor,
		CHECKERBOARD,
		cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
	
	if ret == True:
		# if corners found
		objpoints.append(objp3d)
		corners2 = cv2.cornerSubPix(grayColor, corners, (11, 11), (-1, -1), criteria)
		imgpoints.append(corners2)

		# Draw and display the corners
		image = cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)

		cv2.imshow('img', image)

	cv2.waitKey(0)

print(f"Number of valid images found with corners: {len(imgpoints)}")

ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(objpoints, imgpoints, image_shape, None, None)

cv2.destroyAllWindows()

print("Calibration Successful. Reprojection Error:", ret)

# Displaying required output
print(" Camera matrix:")
print(matrix)

print("\n Distortion coefficient:")
print(distortion)

print("\n Rotation Vectors:")
print(r_vecs)

print("\n Translation Vectors:")
print(t_vecs)

# Save Calibration to file
print("Saving to file...")
filename = "camera_calibration.yaml"
cv_file = cv2.FileStorage(filename, cv2.FILE_STORAGE_WRITE)

cv_file.write("Reprojection Error", ret)
cv_file.write("camera_matrix", matrix)
cv_file.write("distortion_coefficients", distortion)
cv_file.write("reprojection_error", ret)
cv_file.write("Dimensions", dimension)
cv_file.release()
print(f"Calibration data stored in {filename}")