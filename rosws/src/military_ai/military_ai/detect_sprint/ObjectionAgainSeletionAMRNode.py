# hangman_game/word_service.py

import rclpy
from rclpy.node import Node
from military_interface.srv import MoveCar


class ObjectionSeletionAMRNode(Node):

    def __init__(self,mainServer):
        super().__init__("word_service")
        self.move_car_srv = self.create_service(MoveCar, 'move_car', self.move_car_callback)
        self.get_logger().info('MoveCar Service Ready')
        self.mainServer = mainServer
        # self.ask_right_cli = self.create_client(AskRight, 'ask_right')
        # while not self.ask_right_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for AskRight service...')
        # self.req = AskRight.Request()
        # self.get_logger().info('AskRight service Ready')
        # self.ask_right_send_request()
        
    def move_car_callback(self, request, response):
        self.get_logger().info(f'Received MoveCar request: {request}')
        self.detection_result = self.mainServer.get_detection_result()
        # --- 제대로 되고 있나 확인해서 트래킹하는 값 정하기
        response.move = True  # 요청 성공 여부 (예시)
        self.get_logger().info(f'check detection_result in function {self.detection_result}')
        # 최대 컨피던스 선택
        if self.detection_result != None and len(self.detection_result) > 0:
            self.detection_result.sort(key=lambda x: x[6], reverse=True)
            self.follow_car_ID = self.detection_result[0][5]
            self.AMR_STATUS = 0
            response.move = False  # 요청 성공 여부 (예시)
        return response
    def ask_right_send_request(self):
        response = self.ask_right_cli.call(self.req)
        self.ask_right_response = None  # 이전 응답 초기화
        self.ask_right_event.clear()  # 이벤트 초기화
        self.ask_right_response_callback(response)
        self.ask_right_event.wait()  # 이벤트 초기화
        return self.ask_right_event


    def ask_right_response_callback(self, response):
        try:
            self.get_logger().info(f'Received AskRight response: {response}')
            self.service_response = response.right  # 응답 저장
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.service_response = False  # 에러 발생 시 False 반환
        finally:
            self.ask_right_event.set()  # 이벤트 트리거 (대기 중인 쓰레드 깨움)
            

def main(args=None):
    rclpy.init(args=args)
    word_service = RequestSolveNode()
    rclpy.spin(word_service)
    word_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
