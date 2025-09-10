
'''
	클라이언트 서비스 요청은 비동기 방식으로 처리됨
		-> call_async() 함수를 호출하면 요청이 전송되고 함순는 바로 반환됨.
		-> 서버의 응답이 완료되었는지를 따로 확인할 방법이 필요함
	
	Future 객체
		- 비동기 작업의 결과를 나중에 받아보기 위한 객체로 내부 상태가 계속 변화함
		- 서버 응답이 들어오면 Future 상태가 변화하고, 이를 통해 클라이언트는 서버로부터 응답을 받았는지
		  체크할 수 있음
		- 주요 메서드
			- done(): 결과가 준비되었느지(서버 응답이 도착했는지) 여부를 변환
			- result(): 결과값을 반환
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

future = client.call_async(request)
while not future.done():
	rclpy.spin_once(node)
	print(future.done(), future.result())

node.destroy_node()
rclpy.shutdown()

