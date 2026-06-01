"""!
@file	Scanner.py
@author Levi Smith
@brief	Contains the Scanner class used to gether surface data from the sample via laser triangulation
"""

import numpy as np
import threading
import time
import cv2

# Conservative Values
#LOW_RED_LOW = np.array((0,80,130))
#LOW_RED_HIGH = np.array((10,255,255))
#HIGH_RED_LOW = np.array((150,0,120))
#HIGH_RED_HIGH = np.array((180,255,255))

# Controlled Enviorment Values
LOW_RED_LOW = np.array((0,30,80))
LOW_RED_HIGH = np.array((20,255,255))
HIGH_RED_LOW = np.array((150,0,120))
HIGH_RED_HIGH = np.array((180,255,255))

MIN_TOTAL_INTENSITY = 50

class Scanner:

	# @brief 			- the constructor for the scanner class
	# @param H			- the height of the camera from the laser line
	# @param cam_angle	- the angle down towards the laser line relative to the path of propagation of the laser beam (in degrees)
	# @param src		- the source for the camera stream
	def __init__(self, H: float, cam_angle: float, src: int = 0):

		self.H = H
		self.cam_angle = cam_angle

		self.src = src
		self.debug_camera_mode = "none"
		self.vcap = None
		self.capture_stopped = False

		# Shared resources
		self.frame = None
		self.hsv_frame = None
		self.processed_frame = None
		self.mask = None
		self.laser_line = None


	# @brief Handles the mouse callback event to print the color at that pixel for color selection
	# @param event	- The event
	# @param x:		- The x cordinate of the click
	# @param y:		- The y cordinate of the click
	# @param flags:	- The flags/special conditions (was the ctl key held down, etc)
	# @param image:	- The image passed via setMouseCallback() parameter
	# @returns None
	def __pick_color(self, event, x, y, flags, image):
		if event == cv2.EVENT_LBUTTONDOWN:
			pixel = self.hsv_frame[y,x]
			hsv_px = cv2.cvtColor(np.uint8([[pixel]]), cv2.COLOR_BGR2HSV)[0][0]
			print(f"HSV values at [{x},{y}]: {hsv_px}")

	def update_capture(self):
		ret, frame = self.vcap.read()

		if not ret:
			print("Error: Failed to recieve camera frames. Exiting...")
			self.capture_stopped = True
			return

		self.frame = frame
		self.hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# Masking out non-laser hsv values
		low_mask = cv2.inRange(self.hsv_frame, LOW_RED_LOW, LOW_RED_HIGH)
		high_mask = cv2.inRange(self.hsv_frame, HIGH_RED_LOW, HIGH_RED_HIGH)
		self.mask = (low_mask + high_mask)
		self.processed_frame = cv2.bitwise_and(frame, frame, mask=(self.mask))
		

	
	# @brief Connects to the camera
	def connect(self):
		print("Connecting to Camera...")
		self.vcap = cv2.VideoCapture(0)

		# Check if the camera correctly opened
		if not self.vcap.isOpened():
			raise(RuntimeError("Failed to open camera"))
		print("Connected to Camera")

		# Setting up cv frame if specified
		if self.debug_camera_mode != "none":
			print("Setting up camera feed")
			cv2.namedWindow("Camera Feed")
			cv2.setMouseCallback('Camera Feed', self.__pick_color)
		
		# Turn off opencv auto exposure
		self.vcap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
		#self.vcap.set(cv2.CAP_PROP_EXPOSURE, -8)
		self.vcap.set(cv2.CAP_PROP_GAIN, 0)
	
	# @brief Displays debug camera
	# @param debug_camera_mode
	def disp_capture(self, debug_camera_mode: str = "raw"):
		self.debug_camera_mode = debug_camera_mode

		# bitwise AND operation used to secure the last character byte only
		if cv2.waitKey(1) & 0xFF == ord('q'):
			self.capture_stopped = True

		# displaying camera feed if specified
		if self.debug_camera_mode == "processed":
			for point in self.laser_line:
				self.processed_frame[point[0],point[1]] = (0,255,0)
			cv2.imshow('Camera Feed', self.processed_frame)
		elif self.debug_camera_mode == "raw":
			cv2.imshow('Camera Feed', self.frame)
	
	def capture_line_data(self):
		current_gframe = cv2.cvtColor(self.processed_frame, cv2.COLOR_BGR2GRAY)
		height, width = current_gframe.shape
		self.laser_line = np.zeros((width), float)
		position_vector = np.arange(0,width)
		# Iterate through collumns and determine line point via weighed average
		for i in range(width):
			if not np.sum(current_gframe[:, i]) < MIN_TOTAL_INTENSITY: # only add point to line if there is enough total intensity in that col
				self.laser_line[i] (i, (1/width) * (np.sum(current_gframe[:, i] * position_vector)/np.sum(position_vector)))
		pass


	# @brief Shuts down the scanner
	def shutdown(self):
		self.capture_stopped = True
		if self.vcap:
			self.vcap.release()
		cv2.destroyAllWindows()
		print("Scanner safely shutdown")

	