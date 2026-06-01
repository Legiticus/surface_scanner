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
	time.sleep(1) # Allow time for device to connect
	device.reset_input_buffer() # Flush input buffer

	print('Sending Init Message to Collector')
	device.write(b'HELLO\n')


	####################
	#	3. Connect to Camera
	####################

	scanner = Scanner(125,35,0)
	scanner.connect()
	
	# Display camera frames
	if debug_camera_mode in ("processed", "raw"):
		print("Displaying Debug Capture")
		while not scanner.capture_stopped:
			scanner.update_capture()
			scanner.capture_line_data()
			scanner.disp_capture(debug_camera_mode)


		
