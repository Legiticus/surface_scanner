"""!
@file	main.py
@author Levi Smith
@brief	The main file for the 3D surface scanning application
"""

import numpy as np
import time
import serial.tools.list_ports
import serial


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


	####################
	#	3. Communicate with device
	####################

	print('Sending Init Message to Collector')
	device.write(b'HELLO\n')

	while True:
		if device.in_waiting > 0:
			line = device.readline()
			# Convert bytes to string
			decoded_line = line.decode('utf-8').strip()
			print(f'Received: \"{decoded_line}\"')
			device.write(b'HELLO\n')
	