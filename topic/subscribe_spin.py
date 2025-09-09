
import rclpy
from turtlesim.msg import Pose

def callback(data):
	print("--->")
	print("/turtle/pose: ", data)
	print("data.x: ", data.x)
	print("data.y: ", data.y)
	
rclpy.init()
node = rclpy.create_node('my_node')
node.create_subscription(Pose, '/turtle1/pose', callback, 10)

rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
