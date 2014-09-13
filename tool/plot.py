from sys import argv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import serial
import struct
import numpy as np
import threading, time

#Serial port defalut settings
port = '/dev/ttyUSB0'
baudrate = 57600
_serial = serial.Serial(port, baudrate, timeout = 1024)

#Please configure the onboard data at here
onboard_data_name = ['accel-x.raw', 'accel-y.raw', 'accel-z.raw',
		     'gyro-x.raw', 'gyro-y.raw', 'gyro-z.raw']
onboard_data_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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

class AnalogPlot:
	def __init__(self, plot_data_max_count):
		self.plot_data_max_count = plot_data_max_count

		#Plot data, use ring buffer data structure
		self.accel_raw_x_data = deque([0.0] * plot_data_max_count)
		self.accel_raw_y_data = deque([0.0] * plot_data_max_count)
		self.accel_raw_z_data = deque([0.0] * plot_data_max_count)
		self.gyro_raw_x_data = deque([0.0] * plot_data_max_count)
		self.gyro_raw_y_data = deque([0.0] * plot_data_max_count)
		self.gyro_raw_z_data = deque([0.0] * plot_data_max_count)
		self.accel_filter_x_data = deque([0.0] * plot_data_max_count)
		self.accel_filter_y_data = deque([0.0] * plot_data_max_count)
		self.accel_filter_z_data = deque([0.0] * plot_data_max_count)
		self.gyro_filter_x_data = deque([0.0] * plot_data_max_count)
		self.gyro_filter_y_data = deque([0.0] * plot_data_max_count)
		self.gyro_filter_z_data = deque([0.0] * plot_data_max_count)

		plt.ion()
		plt.figure(figsize=(14,8))

		#Subplot - accel data
		self.accel_fig = plt.subplot(211)
		plt.xlabel('Time')
		plt.ylabel('Acceleration')
		self.accel_raw_x, = plt.plot(self.accel_raw_x_data, label='raw x-axis', color='red')
		self.accel_raw_y, = plt.plot(self.accel_raw_y_data, label='raw y-axis', color='blue')
		self.accel_raw_z, = plt.plot(self.accel_raw_x_data, label='raw z-axis', color='green')
		self.accel_filter_x, = plt.plot(self.accel_raw_x_data, label='filter x-axis', color='orange')
		self.accel_filter_y, = plt.plot(self.accel_raw_x_data, label='filter y-axis', color='yellow')
		self.accel_filter_z, = plt.plot(self.accel_raw_x_data, label='filter z-axis', color='purple')
		plt.ylim([-4, 4])
		plt.legend()
		plt.grid()
		plt.draw()

		#Subplot - gyro data
		self.gyro_fig = plt.subplot(212)
		plt.xlabel('Time')
		plt.ylabel('Degree per second')
		plt.ylim([-90, 90])
		self.gyro_raw_x, = plt.plot(self.gyro_raw_x_data, label='x-axis', color='red')
		self.gyro_raw_y, = plt.plot(self.gyro_raw_y_data, label='y-axis', color='blue')
		self.gyro_raw_z, = plt.plot(self.gyro_raw_z_data, label='z-axis', color='green')
		self.gyro_filter_x, = plt.plot(self.gyro_raw_x_data, label='filter x-axis', color='orange')
		self.gyro_filter_y, = plt.plot(self.gyro_raw_x_data, label='filter y-axis', color='yellow')
		self.gyro_filter_z, = plt.plot(self.gyro_raw_x_data, label='filter z-axis', color='purple')
		plt.legend()
		plt.grid()
		plt.draw()

	def add_new_data(self, data, value):
		if len(data) < self.plot_data_max_count:
			data.append(val)
		else:
			data.pop()
		data.appendleft(value)

	def update(self):
		#Get the new data and update the ring buffer
		self.add_new_data(self.accel_raw_x_data, np.array(onboard_data_value[0]))
		self.add_new_data(self.accel_raw_y_data, np.array(onboard_data_value[1]))
		self.add_new_data(self.accel_raw_z_data, np.array(onboard_data_value[2]))
		self.add_new_data(self.gyro_raw_x_data, np.array(onboard_data_value[3]))
		self.add_new_data(self.gyro_raw_y_data, np.array(onboard_data_value[4]))
		self.add_new_data(self.gyro_raw_z_data, np.array(onboard_data_value[5]))
		self.add_new_data(self.accel_filter_x_data, np.array(onboard_data_value[6]))
		self.add_new_data(self.accel_filter_y_data, np.array(onboard_data_value[7]))
		self.add_new_data(self.accel_filter_z_data, np.array(onboard_data_value[8]))
		self.add_new_data(self.gyro_filter_x_data, np.array(onboard_data_value[9]))
		self.add_new_data(self.gyro_filter_y_data, np.array(onboard_data_value[10]))
		self.add_new_data(self.gyro_filter_z_data, np.array(onboard_data_value[11]))

		#Plot the new data
		self.accel_raw_x.set_ydata(self.accel_raw_x_data)
		self.accel_raw_y.set_ydata(self.accel_raw_y_data)
		self.accel_raw_z.set_ydata(self.accel_raw_z_data)
		self.gyro_raw_x.set_ydata(self.gyro_raw_x_data)
		self.gyro_raw_y.set_ydata(self.gyro_raw_y_data)
		self.gyro_raw_z.set_ydata(self.gyro_raw_z_data)
		self.accel_filter_x.set_ydata(self.accel_filter_x_data)
		self.accel_filter_y.set_ydata(self.accel_filter_y_data)
		self.accel_filter_z.set_ydata(self.accel_filter_z_data)
		self.gyro_filter_x.set_ydata(self.gyro_filter_x_data)
		self.gyro_filter_y.set_ydata(self.gyro_filter_y_data)
		self.gyro_filter_z.set_ydata(self.gyro_filter_z_data)

		plt.draw()

class SerialReadThread(threading.Thread):
	def run(self):
		while True:
			if(read_new_data() != 'success'):
				continue

class PrintOnboardData(threading.Thread):
	def run(self):
		while(False):
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
			print 'Accel filter data'
			print 'x:%f' %onboard_data_value[6]
			print 'y:%f' %onboard_data_value[7]
			print 'z:%f' %onboard_data_value[8]
			print 'Gyro filter data'
			print 'x:%f' %onboard_data_value[9]
			print 'y:%f' %onboard_data_value[10]
			print 'z:%f' %onboard_data_value[11]

class DataPlotThread(threading.Thread):
	def run(self):
		analog_plot = AnalogPlot(200)

		while True:
			analog_plot.update()


SerialReadThread().start()
PrintOnboardData().start()
DataPlotThread().start()
