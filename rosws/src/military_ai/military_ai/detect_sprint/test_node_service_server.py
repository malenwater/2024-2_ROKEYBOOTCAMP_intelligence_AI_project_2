import rclpy
from rclpy.node import Node
from military_interface.srv import AskRight  # 'military_interface' 패키지의 AskRight 서비스 파일을 import
import time
class StringArrayService(Node):
    def __init__(self):
        super().__init__('string_array_service')  # 노드 이름
        # 'ask_right' 서비스 이름으로 AskRight 서비스 생성
        self.srv = self.create_service(AskRight, 'ask_right', self.ask_right_callback)

    def ask_right_callback(self, request, response):
        # 요청에서 문자열 배열 데이터 받기
        data = request.data
        self.get_logger().info(f'Received data: {data}')
        time.sleep(10)
        # 응답에서 boolean 값 설정
        response.right = True  # 예시로 right 값을 True로 설정
        return response

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = StringArrayService()  # StringArrayService 노드 실행
    rclpy.spin(node)  # 노드 실행 대기

    # 노드 종료 처리
    rclpy.shutdown()

if __name__ == '__main__':
    main()
