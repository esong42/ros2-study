from my_first_package_msgs.srv import MultiSpawn
import rclpy
from rclpy.node import Node

class MultiSpawning(Node):
	
	def __init__(self):
		super().__init__('multi_spawn')
		self.server = self.create_service(MultiSpawn, 'multi_spawn', self.callback_service)

	def callback_service(self, request, response):
		print('Request: ', request)

		response.x = [1., 2., 3.]
		response.y = [10., 20.]
		response.theta = [100., 200., 300.]

		return response

def main(args=None):
	rclpy.init(args=args)
	multi_spawn = MultiSpawning()

	rclpy.spin(multi_spawn)
	multi_spawn.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

