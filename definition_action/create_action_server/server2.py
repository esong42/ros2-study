import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_first_package_msgs.action import DistTurtle
import time

class DistTurtleServer(Node):
	def __init__(self):
		super().__init__('dist_turtle_action_server')
		self.action_server = ActionServer(
			self,
			DistTurtle,
			'dist_turtle',
			self.execute_callback)
		
	def execute_callback(self, goal_handle):
		
		feedback_msg = DistTurtle.Feedback()

		for n in range(0, 3):
			feedback_msg.remained_dist = float(n)
			goal_handle.publish_feedback(feedback_msg)
			time.sleep(0.5)

		goal_handle.succeed()
		result = DistTurtle.Result()
		return result

def main(args=None):
	rclpy.init(args=args)
	dist_turtle_action_server = DistTurtleServer()
	rclpy.spin(dist_turtle_action_server)
	dist_turtle_action_server.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()