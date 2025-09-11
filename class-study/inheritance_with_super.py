
'''
	파생 클래스에서 기반 클래스의 메서드를 호출할 때 super()를 사용한다.

	아래 Cat과 Dog 클래스는 __init__에서 공통 속성을 유지하면서, 각자 고유 속성을 추가해야 했다.
	이를 위해 __init__을 오버라이딩하고, super()를 사용해 기반 클래스의 __init__을 호출한 다음에
	파생 클래스에서 고유 속성 name을 추가해주었다.
'''

class Animal:
	def __init__(self, food, sleep):
		self.food = food
		self.sleep = sleep

class Cat(Animal):
	def __init__(self, food, sleep):
		super().__init__(food, sleep)
		self.name = 'cat'

class Dog(Animal):
	def __init__(self, food, sleep):
		super().__init__(food, sleep)
		self.name = 'dog'

dog = Dog(100, 100)
print(dog.__dict__)

cat = Cat(50, 50)
print(cat.__dict__)

