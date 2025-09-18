import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_first_package_msgs.msg import CmdAndPoseVel

class CmdAndPose(Node): 

	def __init__(self):
		super().__init__('turtle_cmd_pose')

		self.cmd_pose = CmdAndPoseVel()
		self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.callback_pose, 10)
		self.sub_cmdvel = self.create_subscription(Twist, '/turtle1/cmd_vel', self.callback_cmd, 10)

		self.timer_period = 1.0
		self.publisher = self.create_publisher(CmdAndPoseVel, '/cmd_and_pose', 10)
		self.timer = self.create_timer(self.timer_period, self.callback_timer)

	def callback_pose(self, msg):
		self.cmd_pose.pose_x = msg.x
		self.cmd_pose.pose_y = msg.y
		self.cmd_pose.linear_vel = msg.linear_velocity
		self.cmd_pose.angular_vel = msg.angular_velocity
		
	def callback_cmd(self, msg):
		self.cmd_pose.cmd_vel_linear = msg.linear.x
		self.cmd_pose.cmd_vel_angular = msg.angular.z
		print(self.cmd_pose)

	def callback_timer(self):
		self.publisher.publish(self.cmd_pose)

def main():
	rclpy.init()
	turtle_cmd_pose_node = CmdAndPose()
	
	rclpy.spin(turtle_cmd_pose_node)
	turtle_cmd_pose_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
