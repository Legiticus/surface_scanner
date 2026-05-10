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
	
	# Display camera frames
	while True:
		ret, frame = cap.read()

		if not ret:
			print("Error: Failed to recieve camera frames. Exiting...")
			exit()
		
		cv2.imshow('Cam Feed', frame)

		# bitwise AND operation used to secure the last character byte only
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()