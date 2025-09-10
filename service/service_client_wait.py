
'''
	서비스 클라이언트는 해당 서비스를 제공하는 서버 노드가 실행 중이어야 정상적으로 요청을 보낼 수 있음
	서비가 아직 실행 중이지 않은 상태에서 클라이언트가 요청을 보내면, 프로그램이 장시간 멈춰버리거나 요청이
	실패하면서 에러가 발생함
	
	wait_for_service 함수를 사용하여 서버 실행 상태를 확인하여, 서버가 실행 중일 때 요청을 보내면
	위의 문제를 해결할 수 있음
'''

import rclpy
from turtlesim.srv import TeleportAbsolute

rclpy.init()
node = rclpy.create_node('my_node')

service_name = '/turtle1/teleport_absolute'
client = node.create_client(TeleportAbsolute, service_name)

request = TeleportAbsolute.Request()
request.x = 2.0
request.y = 2.0

while not client.wait_for_service(timeout_sec=1.0):
	print("서비스 아직 준비 안 됨")
'''
	wait_for_service(timeout_sec=1.0)
		- 지정한 시간 동안 서버가 준비되었는지 확인
		- 준비되지 않았다면 flase 반환	
'''

client.call_async(request)
rclpy.spin_once(node)

node.destroy_node()
rclpy.shutdown()

