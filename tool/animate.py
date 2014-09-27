from visual import *
import numpy as np
import threading
import serial
import struct

#Serial port defalut settings
port = '/dev/ttyUSB0'
baudrate = 57600
_serial = serial.Serial(port, baudrate, timeout = 1024)

def serial_read_new_data(data, data_count):
	while True:
		buffer = []
		checksum = 0

		if _serial.read() != '@': #Wait for start byte
				continue

		#Receive the second byte, payload count
		payload_count, =  struct.unpack("B", _serial.read(1))

		for i in range(0, payload_count):
			#Get the new byte
			buffer.append(_serial.read())

			#Cast the receive byte then calculate the checksum
			buffer_checksum ,= struct.unpack("B", buffer[i])
			checksum ^= buffer_checksum

		#Receive the checksum byte then exam it
		checksum_byte ,= struct.unpack("B", _serial.read())
		if checksum_byte != checksum:
			return 'fail'

		for i in range(0, data_count):
			#Prepare the byte data (float = 4 bytes)
			float_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
			#Update the unpack float data
			data[i] = np.asarray(struct.unpack("f", float_data))

		break

	return 'success'

data_count = 20
data = [0.0 for i in range(0, data_count)]

board = box(length=51, height=3, width=35)
board.color = color.blue

class SerialReadThread(threading.Thread):
        def run(self): 
                while True:     
                        serial_read_new_data(data, data_count)

def attitude_3d_animate():
	#Attitude (Degree)
	attitude_roll = 0.0
	attitude_pitch = 0.0
	attitude_yaw = 0.0
	last_attitude_roll = 0.0
	last_attitude_pitch = 0.0
	last_attitude_yaw = 0.0


	while(True):
		attitude_roll = data[15]
		attitude_pitch = data[16]
		attitude_yaw = data[17]

		rotate_roll = attitude_roll - last_attitude_roll
		rotate_pitch = attitude_pitch - last_attitude_pitch
		rotate_yaw = attitude_yaw - last_attitude_yaw

		board.rotate(angle=radians(rotate_roll), axis=(0, 0, 1))
		board.rotate(angle=radians(rotate_pitch), axis=(1, 0, 0))
		board.rotate(angle=radians(-rotate_yaw), axis=(0, 1, 0))

		last_attitude_roll = attitude_roll
		last_attitude_pitch = attitude_pitch
		last_attitude_yaw = attitude_yaw

		rate(500);

SerialReadThread().start()
attitude_3d_animate()
