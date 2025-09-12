# [Package] 패키지에 publisher 노드 추가

## 완성된 my_subscrition.py

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlesimPublisher(Node):

	def __init__(self):
		super().__init__('turtlesim_publisher')
		self.publisher = self.create_publisher(
			Twist,
			'/turtle1/cmd_vel',
			10
		)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def timer_callback(self):
		msg = Twist()
		msg.linear.x = 2.0
		msg.angular.z = 2.0
		self.publisher.publish(msg)		

def main(args=None):
	rclpy.init(args=args)

	turtlesim_publisher = TurtlesimPublisher()
	rclpy.spin(turtlesim_publisher)

	turtlesim_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
```
<br><br>

## 새 노드 추가 후 재빌드해주기
<br>

**1. setup.py 에 새 노드 등록해주기**

위의 완성한 노드를 실행되게 하기 위해선 src/setup.py 에 새 노드로 등록해주어야 한다. 

```python
entry_points={
    'console_scripts': [
        'my_first_node = my_first_package.my_first_node:main',
        'my_subscriber = my_first_package.my_subscriber:main',
    ],
},
```

- 노드 이름과 해당 노드의 파일에서 시작하는 함수(현재 위의 코드에서는 main)를 지정해주면 된다.
<br><br>

**2. 재빌드**

```python
colcon build
```
<br>

**3. 환경변수 갱신**

```python
source install/setup.bash
```
<br>

**4. 실행 확인**

```python
ros2 run my_first_package my_subscriber
```

