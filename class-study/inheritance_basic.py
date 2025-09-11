
'''
	여러 클래스에서 공통된 속성이나 메서드가 있을 때, 하나의 기반 클래스를 정의하고 상속받으면
	중복 코드를 줄일 수 있다.

	아래의 sin, cos 그리는 클래스의 경우 계산 로직 일부만 다르고 대부분의 코드는 동일하다. 다른 동작이
	필요한 clac_wave() 함수만 파생 클래스에서 오버라이딩하여 다르게 구현해주면 된다.
'''

'''
	<python 상속 기본 문법>

	1. 파생 클래스 정의 시 기반 클래스 이름 명시
		-> class Derived(Base)

	2.  파생 클래스에서 기반 클래스 메서드 오버라이딩
		-> 기반 클래스와 같은 이름으로 함수 정의하고 새롭게 구현해주면 됨
'''

import matplotlib.pyplot as plt
import numpy as np

# 기반 클래스
class DrawSinusoidal():
	def __init__(self, amp, freq, bias, end_time):
		self.amp = amp
		self.freq = freq
		self.bias = bias
		self.end_time = end_time

	def calc_wave(self):
		raise NotImplementedError

	def draw_wave(self):
		y = self.calc_wave()
		plt.figure(figsize=(12, 6))
		plt.plot(self.t, y)
		plt.grid()
		plt.show()

# 파생 클래스 1: 사인
class DrawSin(DrawSinusoidal):
	def calc_wave(self):
		# 메서드 오버라이딩
		self.t = np.arange(0, self.end_time, 0.01)
		return self.amp * np.sin(2 * np.pi * self.freq * self.t) + self.bias

# 파생 클래스 2: 코사인
class DrawCos(DrawSinusoidal):
	def calc_wave(self):
		# 메서드 오버라이딩
		t = np.arange(0, self.end_time, 0.01)
		return self.amp * np.cos(2 * np.pi * self.freq * self.t) + self.bias
		

sin_wave = DrawSin(1, 1, 0, 3)
cos_wave = DrawCos(1, 1, 0, 3)

sin_wave.draw_wave()
cos_wave.draw_wave()

