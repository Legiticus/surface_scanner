"""!
@file	Scanner.py
@author Levi Smith
@brief	Contains the Scanner class used to gether surface data from the sample via laser triangulation
"""

import numpy as np
import matplotlib.pyplot as plt
import cv2

# Conservative Values
LOW_RED_LOW = np.array((0,80,130))
LOW_RED_HIGH = np.array((10,255,255))
HIGH_RED_LOW = np.array((150,0,120))
HIGH_RED_HIGH = np.array((180,255,255))

# Controlled Enviorment Values
#LOW_RED_LOW = np.array((0,30,80))
#LOW_RED_HIGH = np.array((20,255,255))
#HIGH_RED_LOW = np.array((150,0,120))
#HIGH_RED_HIGH = np.array((180,255,255))

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

		# Position data
		self.current_z = 0

		# Shared resources
		self.frame = None
		self.hsv_frame = None
		self.processed_frame = None
		self.mask = None
		self.laser_line = None

		self.total_points = []

		# Calibration data
		self.fx = 0
		self.fy = 0
		self.cu = 0
		self.cv = 0
		self.__read_calibration_data()

	def getHeight(self):
		return self.current_z
	
	def setHeight(self, height: float):
		self.current_z = height

	def moveHeight(self, inc: float):
		self.current_z += inc
		return self.current_z


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
	
	def __read_calibration_data(self):
		path = "./Calibration/camera_calibration.yaml"
		cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
		cam_matrix = cv_file.getNode("camera_matrix").mat()
		cv_file.release()
		self.fx = cam_matrix[0,0]
		self.fy = cam_matrix[1,1]
		self.cu = cam_matrix[0,2]
		self.cv = cam_matrix[1,2]

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
				row = int(point[1])
				col = int(point[0])
				self.processed_frame[(row-1):(row+2),col] = (0,255,0)
			cv2.imshow('Camera Feed', self.processed_frame)
		elif self.debug_camera_mode == "raw":
			cv2.imshow('Camera Feed', self.frame)
	
	# @brief takes in a weighted processed grayscale frame and analyses it to find the laser line
	# @param weighted_frame	- the weighted processed grayscale frame
	# @returns None, but places laser line data in memeber variable laser_line
	def __capture_line_data(self, weighted_frame):
		current_gframe = weighted_frame
		height, width = current_gframe.shape
		self.laser_line = np.zeros((width,2))
		position_vector = np.arange(0,height)
		# Iterate through collumns and determine line point via weighed average
		for i in range(width):
			if not np.sum(current_gframe[:, i]) < MIN_TOTAL_INTENSITY: # only add point to line if there is enough total intensity in that col

				numerator = np.sum(current_gframe[:, i] * position_vector)
				denomitator = np.sum(current_gframe[:, i])

				if denomitator != 0: # avoid division by zero (note that zero is the default value for the array)
					self.laser_line[i] = ((i), (numerator/denomitator))
				else:
					self.laser_line[i] = (i,0)
	
	# @brief processed the line data and returns an array of points
	# @returns the array of points on the surface
	def __process_line_data(self):
		# Part 1: transform cordinate system
		width = self.laser_line.shape[0]
		points = np.zeros((width, 3), float)

		# Calculate commonly used calculations to save time and inprove readability
		sint = np.sin(self.cam_angle)
		cost = np.cos(self.cam_angle)
		hz = self.H - self.current_z

		# calculates x cord in real world space
		def x(u,v):
			num = self.fy * cost - (v - self.cv) * sint
			denom = self.fy * sint + (v - self.cv) * cost
			if abs(denom) < 1e-14: raise ValueError("divide by zero")
			return hz * num/denom

		# calculates y cord in real world space
		def y(u,v):
			num = hz * (u - self.cu)
			denom = self.fy * sint + (v - self.cv)
			if abs(denom) < 1e-14: raise ValueError("divide by zero")
			return -self.fy/self.fx * num/denom

		for i in range(width):
			# Division by zero handled
			try:
				X = x(self.laser_line[i,0], self.laser_line[i,1])
				Y = y(self.laser_line[i,0], self.laser_line[i,1])
				points[i,:] = (X, Y, self.current_z)
			except ValueError:
				points[i,:] = float('nan')
		
		return points

	
	# @brief takes a sample of the laser line by taking the average values over multiple images then processing weighted image to find the line
	# @param sample_count	- the number of samples to use
	# @returns None, but adds points to total points
	def sample_laser_line(self, sample_count: int = 1):
		if (sample_count < 1): raise ValueError("cannot sample less than 1 samples")
		self.update_capture()
		height, width, chn = self.processed_frame.shape # Channel is unused
		weighted_frame = np.zeros((height,width), float)
		for i in range(sample_count):
			weighted_frame += cv2.cvtColor(self.processed_frame, cv2.COLOR_BGR2GRAY)
			self.update_capture()
		weighted_frame /= sample_count
		self.__capture_line_data(weighted_frame)
		points = self.__process_line_data()
		for i in range(points.shape[0]):
			if not np.isnan(points[i,:]).any():
				self.total_points.append(points[i,:])

	# @brief exports the point data
	# @return a copy of the points
	def export_points(self):
		return self.points.copy()
	
	# @brief exports the point data
	# @return a copy of the points
	def plot_points(self):

		# Unpack points
		X, Y, Z = zip(*self.total_points)

		fig = plt.figure()
		ax = fig.add_subplot(projection='3d')
		ax.scatter(X, Y, Z)
		ax.set_xlabel("X (mm)")
		ax.set_ylabel("Y (mm)")
		ax.set_zlabel("Z (mm)")
		plt.show()

	# @brief Shuts down the scanner
	def shutdown(self):
		self.points = None
		self.capture_stopped = True
		if self.vcap:
			self.vcap.release()
		cv2.destroyAllWindows()
		print("Scanner safely shutdown")

	