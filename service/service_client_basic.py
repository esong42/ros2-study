

import rclpy
from turtlesim.srv import TeleportAbsolute

rclpy.init()
node = rclpy.create_node('my_node')

service_name = '/turtle1/teleport_absolute'
client = node.create_client(TeleportAbsolute, service_name)

request = TeleportAbsolute.Request()
request.x = 2.0
request.y = 2.0
'''
	서비스 요청 메세지 형식 확인하려면
		- ros2 interface show turtlesim.srv.TeleportAbsolute
		- print(request)
'''

client.call_async(request)
rclpy.spin_once(node)

node.destroy_node()
rclpy.shutdown()

