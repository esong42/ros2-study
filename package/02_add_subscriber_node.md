# [Package] 패키지에 subscriber 노드 추가

## 완성된 my_subscrition.py

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlesimSubscriber(Node):

	def __init__(self):
		super().__init__('turtlesim_subscriber')
		self.subscription = self.create_subscription(
			Pose,
			'/turtle1/pose',
			self.callback,
			10)
		self.subscription
	
	def callback(self, msg):
		print("X: ", msg.x, ", Y: ", msg.y)

def main(args=None):
	rclpy.init(args=args)

	turtlesim_subscriber = TurtlesimSubscriber()
	rclpy.spin(turtlesim_subscriber)

	turtlesim_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

```
<br>

## 생성된 subscription 객체를 멤버 변수로 저장해주는 이유

클래스를 따로 만들지 않고 `rclpy.node.Node` 클래스를 사용했을 때는 subscription을 따로 저장하지 않았는데, 클래스를 따로 생성할 때는 subscription 생성 후에 객체를 저장한 변수를 만들고 해당 변수에 저장을 해주어야 한다.<br><br>

```python
node = rclpy.create_node('my_node')
node.create_subscription(...)
```

```bash
self.subscription = self.create_subscription(...)
```
<br>

**가비지 컬렉션에 의한 삭제 막기**

python은 자동으로 메모리 관리를 해주는 가비지 컬렉션을 사용한다. 참조가 없는 객체는 사용하지 않는다고 판단하여가비지 컬렉션에 의해 삭제가 될 수 있다.

그래서 create_subscription 함수만 호출하고 반환값을 저장하지 않으면 가비지 컬렉션이 생성된 subscription 객체를 삭제할 수 있음<br>
→ **create_subscription에 의해 생성된 객체를 변수에 저장해줌으로써 삭제를 막을 수 있다!**<br>
→ 근데 단순히 GC에 의해 삭제되는 것을 방지하기 위해 생성한 변수이니 실제로 코드에선 사용하지 않으니 컴파일 시에 unused variable warning 이 발생함.<br>
→  `self.subscription` 코드 추가해서 warning 막아주기!
<br><br>

**rclpy.node.Node 클래스만 사용해 스크립트에서 괜찮았던 이유**

rclpy.onde.Node 클래스 내부적으로 subscription을 관리하기 때문에 객체 참조가 유지됨<br>
→ 가비지 컬렉션에 의해 삭제되지 않음
<br><br>

**‘내가 정의한 클래스 + 클래스 외부에서 create_subscription 호출’ 상황에서 삭제를 피하기 위해선?**

만약 직접 정의한 클래스를 사용하는데 클래스 외부에서 create_subscription 함수를 호출하는 경우라면, 가비지 컬렉션에 의해 삭제되지 않도록 하기 위해서 **참조를 유지해주어야 한다**.

아래와 같이 반환값을 변수에 저장해주면 된다(**변수를 사용하진 않겠지만 참조를 유지하는 게 중요**). 이렇게 해주면 클래스 외부에서도 콜백을 안전하게 유지할 수 있다.<br>
```python
turtlesim_subscriber = TurtlesimSubscriber()
sub = turtlesim_subscriber.create_subscription(...)
```
<br><br>

## rclpy.init 함수에 args 인자를 전달하는 이유
<br>

```python
ros2 run my_package my_node --ros-args -r __node:=new_name
```

위와 같이 ros2에서 노드를 실행할 때 `--ros-args` 옵션을 사용하면 로그 레벨, 노드 이름 등 다양한 옵션을 지정해줄 수 있다.

해당 옵션들이 실제로 적용되려며면 rclpy.init 함수에 인자를 전달해주어야 한다. 만약 전달하지 않으면 기본값으로 노드가 실행된다.
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

