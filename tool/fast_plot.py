import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

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
		self.line.append(plt.plot(self.analog_data[self.current_line_count], \
			label=label_name, color=line_color)[0])
	
	def show_subplot(self):
		plt.grid()
		plt.legend()

	def animate(self, index):
		self.line[index].set_ydata(self.analog_data[index].data)
		return self.line[index],
			
	def __init__(self, figure_count, line_count, analog_data):
		figure = plt.figure()
		self.line = []
		self.current_line_count = 0
		self.analog_data = analog_data

		plt.ion()
		plt.figure(figsize=(14,8))

		plt.subplot(211)
		plt.ylabel('Acceleration (g)')
		plt.ylim([-4, 4])
		self.create_line('x axis (raw data)', 'red')		
		self.create_line('y axis (raw data)', 'blue')		
		self.create_line('z axis (raw data)', 'green')		
		self.create_line('x axis (filter data)', 'orange')		
		self.create_line('y axis (filter data)', 'yellow')		
		self.create_line('z axis (filter data)', 'blue')		
		self.show_subplot()

		plt.subplot(212)
		plt.ylabel('Degree per second (dps)')
		plt.ylim([-2000, 2000])
		self.create_line('x axis (raw data)', 'red')		
		self.create_line('y axis (raw data)', 'blue')		
		self.create_line('z axis (raw data)', 'green')		
		self.create_line('x axis (filter data)', 'orange')		
		self.create_line('y axis (filter data)', 'yellow')		
		self.create_line('z axis (filter data)', 'blue')		
		self.show_subplot()

		animation.FuncAnimation(figure, self.animate, np.arange(0, 200), \
			interval=0, blit=True)

		plt.draw()

def main():
	analog_data = [AnalogData(200) for i in range(0, 12)]

	analog_plot = AnalogPlot(2, 12, analog_data)

	while True:
		pass

main()
