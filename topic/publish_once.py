
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('my_node')

msg = Twist()
msg.linear.x = 1.0
msg.angular.z = 1.57

pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
pub.publish(msg)

node.destroy_node()
rclpy.shutdown()