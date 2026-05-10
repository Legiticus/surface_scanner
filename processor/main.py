"""!
@file	main.py
@author Levi Smith
@brief	The main file for the 3D surface scanning application
"""

import numpy as np
import time
import serial.tools.list_ports
import serial
import cv2

LOW_RED_LOW = np.array((0,80,130))
LOW_RED_HIGH = np.array((10,255,255))
HIGH_RED_LOW = np.array((150,0,120))
HIGH_RED_HIGH = np.array((180,255,255))

# @brief Handles the mouse callback event to print the color at that pixel for color selection
# @param event	- The event
# @param x:		- The x cordinate of the click
# @param y:		- The y cordinate of the click
# @param flags:	- The flags/special conditions (was the ctl key held down, etc)
# @param image:	- The image passed via setMouseCallback() parameter
# @returns None
def pick_color(event, x, y, flags, image):
	if event == cv2.EVENT_LBUTTONDOWN:
		pixel = hsv_frame[y,x]
		hsv_px = cv2.cvtColor(np.uint8([[pixel]]), cv2.COLOR_BGR2HSV)[0][0]
		print(f"HSV values at [{x},{y}]: {hsv_px}")


if __name__ == "__main__":

	####################
	#	1. Locate Device
	####################

	print('Application Starting...')
	print('Locating Collector...')
	print('Checking Ports...')

	devicePort = 0
	ports = serial.tools.list_ports.comports()

	for port in ports:
		print(f'Device: {port.device}, Description: {port.description}')
		if (port.description.__contains__('Arduino')):
			devicePort = port.device
			print(f'Collector located on {port.device}')
			break
	
	if (devicePort == 0):
		print('Failed to locate collector, terminating process')
		exit()
	
	####################
	#	2. Connect to Device
	####################

	print('Connecting to Collector...')
	device = serial.Serial(devicePort, 115200)
	time.sleep(2) # Allow time for device to connect
	device.reset_input_buffer() # Flush input buffer

	print('Sending Init Message to Collector')
	device.write(b'HELLO\n')


	####################
	#	3. Connect to Camera
	####################

	print("Connecting to Camera...")
	cap = cv2.VideoCapture(0)

	# Check if the camera correctly opened
	if not cap.isOpened():
		raise(RuntimeError("Failed to open camera"))
	print("Connected to Camera")

	# Setting up cv frame
	print("Setting up camera feed")
	cv2.namedWindow("Camera Feed")
	cv2.setMouseCallback('Camera Feed', pick_color)
	
	# Display camera frames
	while True:
		ret, frame = cap.read()

		if not ret:
			print("Error: Failed to recieve camera frames. Exiting...")
			exit()
		
		hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# Masking out non-laser hsv values
		low_mask = cv2.inRange(hsv_frame, LOW_RED_LOW, LOW_RED_HIGH)
		high_mask = cv2.inRange(hsv_frame, HIGH_RED_LOW, HIGH_RED_HIGH)
		processed_frame = cv2.bitwise_and(frame, frame, mask=(low_mask + high_mask))
		
		# displaying camera feed
		cv2.imshow('Camera Feed', processed_frame)

		# bitwise AND operation used to secure the last character byte only
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()