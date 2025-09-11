import matplotlib.pyplot as plt
import numpy as np

class DrawSin:
	def __init__(self, amp, freq, bias, end_time):
		self.amp = amp
		self.freq = freq
		self.bias = bias
		self.end_time = end_time

	def calc_sin(self):
		self.t = np.arange(0, self.end_time, 0.01)
		return self.amp * np.sin(2 * np.pi * self.t * self.freq) + self.bias

	def draw_sin(self):
		y = self.calc_sin()
		plt.figure(figsize=(12, 6))
		plt.plot(self.t, y)
		plt.grid()
		plt.show()
		

tmp = DrawSin(1, 1, 0, 3)
print(tmp.__dict__)
tmp.draw_sin()

