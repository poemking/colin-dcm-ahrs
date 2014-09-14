import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import serial
import struct
from collections import deque


#Serial port defalut settings
port = '/dev/ttyUSB0'
baudrate = 57600
_serial = serial.Serial(port, baudrate, timeout = 1024)

class AnalogData:
	def __init__(self, max_count):
		self.max_count = max_count
		self.data = deque([0.0] * max_count)

	def add(self, value):
		if len(self.data) < self.max_count:
			self.data.append(val)
		else:
			self.data.pop()
		self.data.appendleft(value)

class AnalogPlot:
	def create_line(self, label_name, line_color):
		self.line.append(plt.plot(self.analog_data[self.current_line_count].data, \
			label=label_name, color=line_color, animated=True)[0])

		self.current_line_count += 1

	def show_subplot(self):
		plt.grid()
		plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), \
			ncol=3, fancybox=True, shadow=True)

	def animate(self, i):
		for index in range(0, self.line_count):
			self.line[index].set_ydata(self.analog_data[index].data)

		return self.line
			
	def __init__(self, line_count, analog_data):
		self.figure = plt.figure(figsize=(14,8))
		self.line = []
		self.line_count = line_count
		self.current_line_count = 0
		self.analog_data = analog_data
		self.current_line_count = 0

		plt.subplot(211)
		plt.ylabel('Acceleration (g)')
		plt.ylim([-4, 4])
		self.create_line('x axis (raw data)', 'red')		
		self.create_line('y axis (raw data)', 'blue')		
		self.create_line('z axis (raw data)', 'green')		
		self.create_line('x axis (filter data)', 'orange')		
		self.create_line('y axis (filter data)', 'yellow')		
		self.create_line('z axis (filter data)', 'purple')		
		self.show_subplot()

		plt.subplot(212)
		plt.ylabel('Degree per second (dps)')
		plt.ylim([-2000, 2000])
		self.create_line('x axis (raw data)', 'red')		
		self.create_line('y axis (raw data)', 'blue')		
		self.create_line('z axis (raw data)', 'green')		
		self.create_line('x axis (filter data)', 'orange')		
		self.create_line('y axis (filter data)', 'yellow')		
		self.create_line('z axis (filter data)', 'purple')		
		self.show_subplot()

	def show(self):
		ani = animation.FuncAnimation(self.figure, self.animate, np.arange(0, 200), \
			interval=0, blit=True)

		plt.show()

	def read_new_data(self):
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

			for i in range(0, self.line_count):
				#Prepare the byte data (float = 4 bytes)
				float_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
				#Update the unpack float data
				self.analog_data[i].add(np.asarray(struct.unpack("f", float_data)))

			break

		return 'success'

#Analog plot
analog_data = [AnalogData(200) for i in range(0, 12)]
analog_plot = AnalogPlot(12, analog_data)

class SerialReadThread(threading.Thread):
	def run(self):
		while True:
			analog_plot.read_new_data()

SerialReadThread().start()
analog_plot.show()
