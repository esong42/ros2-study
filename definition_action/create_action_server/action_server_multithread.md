# 서버 만들기 - 멀티스레드

## 멀티 스레드 사용해보기

**멀티 스레드 사용해야 하는 이유**

rclpy.spin 함수는 들어온 서비스, 액션, 토픽 이벤트를 콜백 큐에 쌓는다. 하나씩 꺼내서 순차적으로 이벤트를 처리하기 때문에, 현재 이벤트에 대한 콜백 함수가 종료되어야 다음 이벤트에 대한 콜백 함수를 호출할 수 있다.

이번 액션 서버 만들기의 최종 목표는 turtlesim이 움직인 거리를 측정하고, 사용자의 지시에 따라 turtlesim을 구동시키는 것이다. 이를 위해서는 액션 서버에서 토픽을 발행(cmd_vel)도 해야 하고 토픽을 구독(pose)도 해야 한다.

→ 액션 처리 중간중간 pose로 토픽으로부터 발행된 메세지가 필요한데, 이미 액션 콜백함수가 실행중이어서 이 콜백함수가 종료되기 전까지는 pose로부터 발행된 메세지를 받아볼 수 없음
<br><br>

**멀티 스레드 사용해보기**

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

from my_first_package.my_subscriber import TurtlesimSubscriber
from my_first_package.my_publisher import TurtlesimPublisher

def main(args=None):
		rclpy.init(args=args)
		
		sub_node = TurtlesimSubscriber()
		pub_node = TurtlesimPublisher()
	
		executor = MultiThreadedExecutor()
		executor.add_node(sub_node)
		executor.add_node(pub_node)
	
		executor.spin()
	
		try:
				executor.spin()
		finally:
				executor.shutdown()
				sub_node.destroy_node()
				pub_node.destroy_node()
				rclpy.shutdown()

if __name__ == '__main__':
		main()
```

- **Executor**
    
    Executor는 ROS2의 실행 관리를 담당한다.
    
    Executor는 하나 이상의 스레드를 사용하여 수신 메세지 및 이벤트에 대한 구독, 타이머, 서비스 서버, 액션 서버 등의 콜백을 호출한다.
    
    - **single thread executor vs multi thread executor**
        
        single thread executor는 하나의 노드에 대해서만 이벤트 처리가 가능하다.
        
        multi thread executor는 여러 노드에 대한 이벤트를 병렬로 처리할 수 있도록 해준다. 
<br><br>        
    
- **멀티 스레드로 이벤트 관리하기 위해 MultiThreadedExecutor 사용**
    
    
    ```python
    from rclpy.executors import MultiThreadedExecutor 
    ```
    
    MultiThreadedExecutor 를 사용해주기 위해 rclpy.executors 모듈에서 MultiThreadedExecutor import 해주기
    
    ```python
    executor = MultithreadedExecutor()
    
    executor.add_node(sub_node)
    executor.add_node(pub_node)
    
    executor.spin()
    ```
    
    executor에서 관리해줄 노드를 add_node 함수를 통해 추가해주고 spin 돌리면, 추가한 노드들에 대한 이벤트를 병렬로 처리할 수 있다.
<br><br>

- **다른 파일에 정의된 클래스 사용해주기**
    
    ```python
    from my_first_package.my_subscriber import TurtlesimSubscriber
    from my_first_package.my_publisher import TurtlesimPublisher
    
    sub_node = TurtlesimSubscriber()
    pub_node = TurtlesimPublisher()
    ```
<br> 

## 서버 만들기

사용자가 지정해준 거리만큼 turtlesim을 이동시킨 후 멈추도록 하는 서버 만들기

**pose 토픽 구독하여 처리할 TurtlrlSub_Action 클래스 구현하기**

```python
class TurtleSub_Action(TurtlesimSubscriber):
		def __init__(self, ac_server):
				super().__init__()
				self.ac_server = ac_server
	
def callback(self, msg):
		self.ac_server.current_pose = msg
```

- **TurtlesimSubscriber 클래스 상속하기**
    
    TurtleSub_Action 클래스를 통해 pose 토픽으로부터 메세지를 얻어 turtle의 위치와 각도를 얻을 것이다. 이를 위해 예전에 구현했었던 TurtlesimSubscriber를 기반 클래스로 상속하고 있다.
    
    - **TurtlesimSubscriber 클래스**
        
        ```python
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
        ```
        
        pose 토픽 구독하고 얻은 메세지를 출력해준다.
<br><br>        
    
- **기반 클래스 변수를 그대로 가져와주기위해 super 함수로 init 함수 호출해주기**
    
    ```python
    super().__init__('turtlesim_subscriber')
    ```
<br>

- **callback 함수를 재정의하여 현재 액션 서버에 사용할 형태로 바꿔주기**
    
    현재 기반 클래스의 callback 함수는 pose 토픽으로부터 얻은 데이터를 단순히 출력해주고 있다. 받은 메세지 데이터를 DistTurtleServer 객체에서 사용할 수 있도록 넘겨주는 함수로 재정의 해줘야한다.
    
    ```python
    def callback(self, msg):
    		self.ac_server.current_pose = msg
    ```
    

	**위와 같이 재정의해주면, 기반 클래스에서 구독하고 있는 토픽 이벤트를 파생 클래스의 callback에서 처리하게 된다.**
<br><br>

**DistTurtleServer 클래스 구현하기**

```python
def __init__(self):
		super().__init__('dist_turtle_action_server')
		
		self.total_dist = 0
		self.is_first_time = True
		self.current_pose = Pose()
		self.previous_pose = Pose()

		self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.action_server = ActionServer(
				self,
				DistTurtle,
				'dist_turtle',
				self.execute_callback)
```
<br>

```python
def calc_diff_pose(self):
		if self.is_first_time == True:
				self.previout_pose = self.current_pose
				self.is_first_time = False
		
		diff_dist = math.sqrt((self.current_pose.x - self.previous_pose.x)**2 +\
													(self.current_pose.y - self.previous_pose.y)**2)
		
		self.previous_pose = self.current_pose
		
		return diff_dist
```
<br>

```python
def execute_callback(self, goal_handle):
	
		feedback_msg = DistTurtle.Feedback()
	
		msg = Twist()
		msg.linear.x = goal_handle.request.linear_x
		msg.angular.z = goal_handle.request.angular_z
	
		while (True):
				self.total_dist += self.calc_diff_pose()
				feedback_msg.remained_dist = goal_handle.request.dist - self.total_dist
				goal_handle.publish_feedback(feedback_msg)
				self.publisher.publish(msg)
				time.sleep(0.01)
			
				if feedback_msg.remained_dist < 0.2:
						break
		
		goal_handle.succeed()
		result = DistTurtle.Result()
	
		result.pos_x = self.current_pose.x
		result.pos_y = self.current_pose.y
		result.pos_theta = self.current_pose.theta
		result.result_dist = self.total_dist
	
		self.total_dist = 0
		self.is_first_time = True
	
		return result
```
<br>

**main문 멀티스레드 적용하기**

```python
def main(args=None):
		rclpy.init(args=args)

		executor = MultiThreadedExecutor()

		ac = DistTurtleServer()
		sub = TurtleSub_Action(ac)

		executor.add_node(ac)
		executor.add_node(sub)

		try:
				executor.spin()
		finally:
				executor.shutdown()
				ac.destroy_node()
				sub.destroy_node()
				rclpy.shutdown()
```
