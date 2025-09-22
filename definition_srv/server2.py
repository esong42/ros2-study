from my_first_package_msgs.srv import MultiSpawn
from turtlesim.srv import TeleportAbsolute
import rclpy
from rclpy.node import Node

class MultiSpawning(Node):
	
	def __init__(self):
		super().__init__('multi_spawn')
		self.server = self.create_service(MultiSpawn, 'multi_spawn', self.callback_service)
		self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
		self.req_teleport = TeleportAbsolute.Request()

	def callback_service(self, request, response):
		print('Request: ', request)

		self.req_teleport.x = 1.
		self.req_teleport.y = 2.
		self.teleport.call_async(self.req_teleport)

		return response

def main(args=None):
	rclpy.init(args=args)
	multi_spawn = MultiSpawning()

	rclpy.spin(multi_spawn)
	multi_spawn.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
	