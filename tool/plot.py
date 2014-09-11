from sys import argv
import serial
import struct

#Serial port defalut settings
port = '/dev/ttyUSB0'
baudrate = 57600
_serial = serial.Serial(port, baudrate, timeout = 1024)

#Please configure the onboard data at here
onboard_data_name = ['accel-x.raw', 'accel-y.raw', 'accel-z.raw',
		     'gyro-x.raw', 'gyro-y.raw', 'gyro-z.raw']
onboard_data_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
onboard_data_count = 0

def read_new_data():
	while True:
		buffer = []
		checksum = 0

		if(_serial.read() != '@'): #Waiting for start byte
			continue

		#Get the payload count
		payload_count,= struct.unpack("B", _serial.read(1))

		for i in range(0, payload_count):
			#get the new byte (tuple data type)
			buffer.append(_serial.read())
			#Cast the data from tuple to integer
			buffer_checksum ,= struct.unpack("B", buffer[i])
			#Caluculate the checksum
			checksum ^= buffer_checksum

		#Get the checksum byte then exam it
		checksum_byte ,= struct.unpack("B", _serial.read()) 
		if(checksum_byte != checksum):
			return 'fail'

		#Get the onboard data count (float = 4 byte)
		onboard_data_count = payload_count / 4

		for i in range(0, onboard_data_count):
			#Prepare the byte data (float = 4 bytes)
			float_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
			#Unpack the float data
			onboard_data_value[i] = struct.unpack("f", float_data)

		break
	
	return 'success'

def run():
	while True:
		if(read_new_data() != 'success'):
			continue

		print "\033c"

		#Print the data
		print 'Accel raw data'
		print 'x:%f' %onboard_data_value[0]
		print 'y:%f' %onboard_data_value[1]
		print 'z:%f' %onboard_data_value[2]
		print 'Gyro raw data'
		print 'x:%f' %onboard_data_value[3]
		print 'y:%f' %onboard_data_value[4]
		print 'z:%f' %onboard_data_value[5]

run()
