
import rclpy
from geometry_msgs.msg import Twist

cnt = 0

rclpy.init()
node = rclpy.create_node('my_node')

msg = Twist()
msg.linear.x = 1.0
msg.angular.z = 1.57
pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

def callback():
	global cnt
	cnt += 1
	print(cnt)
	pub.publish(msg)
	if cnt > 10:
		raise Exception("Publisher Stop")

timer_period = 0.5
node.create_timer(timer_period, callback)

try:
	rclpy.spin(node)
except Exception as e:
	print("Exception 발생: ", e)
finally:
	node.destroy_node()
	rclpy.shutdown()