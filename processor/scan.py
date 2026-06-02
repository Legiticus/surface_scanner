"""!
@file	scan.py
@author Levi Smith
@brief	The main file for the 3D surface scanning application
"""

import numpy as np
import time
import serial.tools.list_ports
import serial
import cv2
import getopt
import sys

from Scanner import Scanner

SCAN_HEIGHT = 10 #mm
OPTIONS = "c:hv"

is_verbose = False
debug_camera_mode = "none"



if __name__ == "__main__":


	####################
	#	0. Process command-line arguments
	####################

	argv = sys.argv[1:]
	try:
		opts, args = getopt.getopt(argv, OPTIONS)
	except getopt.GetoptError:
		sys.exit(2)
	
	for opt, arg in opts:
		if opt in ("-h", "--help"):
			print("Usage: scan [OPTIONS]")
			print("\t-c  str\t\tConfigures debug camera with mode \"none\", \"processed\" or \"raw\" (\"none\" by default)")
			print("\t-v\t\tEnables verbose output")
			print("\t-h\t\tPrints help text")
			sys.exit(0) # Exit success
		elif opt in ("-v", "--verbose"):
			is_verbose = True
			print("Running scanner in verbose mode")
		elif opt in ("-c", "--debug-camera"):
			if arg == None:
				print("Missing argument")
				sys.exit(1) # Exit failure
			elif arg not in ("none", "raw", "processed"):
				print("Invalid camera mode. Try -h for help")
				sys.exit(1) # Exit failure
			else:
				debug_camera_mode = arg



	####################
	#	1. Locate Device
	####################

	print('Application Starting...')
	print('Locating Collector...')
	print('Checking Ports...')

	devicePort = 0
	ports = serial.tools.list_ports.comports()

	for port in ports:
		if is_verbose: print(f'Device: {port.device}, Description: {port.description}')
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
	device.reset_output_buffer()

	print('Sending ZERO Message to Controller')
	device.write(b'ZERO\n')

	####################
	#	3. Connect to Camera
	####################

	scanner = Scanner(125,35,0)
	scanner.connect()

	# Getting response from Controller
	raw_data = device.readline()
	line = raw_data.decode('utf-8').strip()
	words = line.split(maxsplit=1)
	if words[0] != "ZERO_SUCCESS":
		print("COLLECTOR:\t", words)
		print("Error zeroing scan head, exiting...")
		exit(1)
	else:
		print("COLLECTOR:", words)

	####################
	#	4. Display Feed and Take Measurements
	####################

	scanner.setHeight(0)

	while scanner.getHeight() < SCAN_HEIGHT:
		print(f"Current Scan Height: {scanner.getHeight()} mm")
		# Sample line over 10 frames
		scanner.sample_laser_line(10)

		# Display debug camera
		if debug_camera_mode in ("processed", "raw") and not scanner.capture_stopped:
			scanner.disp_capture(debug_camera_mode)
		
		# Tell controller to move scan head 0.1 mm
		inc = 0.1
		scanner.moveHeight(inc)
		device.write(b'MOVE 0.1')
		raw_data = device.readline()
	
	scanner.plot_points()
	scanner.shutdown()

	print("--------------------------FINISHED--------------------------")



		
