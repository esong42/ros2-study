
import rclpy
from turtlesim.msg import Pose

cnt = 0

def callback(data):
	global cnt
	cnt += 1
	print("[", cnt, "]: ", data.x, ", ", data.y)
	if cnt > 3:
		raise Exception("Subscription Stop")
		

rclpy.init()
node = rclpy.create_node("my_node")
node.create_subscription(Pose, '/turtle1/pose', callback, 10)

try:
	rclpy.spin(node)
except Exception as e:
	print("Exception 발생: ", e)
finally:
	node.destroy_node()
	rclpy.shutdown()