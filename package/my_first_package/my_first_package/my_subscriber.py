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

