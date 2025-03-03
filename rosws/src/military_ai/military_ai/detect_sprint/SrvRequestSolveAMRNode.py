# hangman_game/word_service.py

import rclpy
from rclpy.node import Node
from military_interface.srv import MoveCar


class SrvRequestSolveAMRNode(Node):

    def __init__(self,mainServer):
        super().__init__("SrvRequestSolveAMRNode")
        self.get_logger().info('SrvRequestSolveAMRNode start')
        self.move_car_srv = self.create_service(MoveCar, 'move_car', self.move_car_callback)
        self.mainServer = mainServer
        self.get_logger().info('SrvRequestSolveAMRNode end')
        
    def move_car_callback(self, request, response):
        self.get_logger().info(f'Received MoveCar request: {request}')
        self.detection_result = self.mainServer.get_detection_result()
        # --- 제대로 되고 있나 확인해서 트래킹하는 값 정하기
        response_data = []
        self.get_logger().info(f'check detection_result in function {self.detection_result}')
        # 최대 컨피던스 선택
        if self.detection_result != None and len(self.detection_result) > 0:
            self.detection_result.sort(key=lambda x: x[6], reverse=True)
            for obj in self.detection_result:
                response_data.append(str(int(obj[4])))
                response_data.append(str(int(obj[5])))

        self.get_logger().info(f'check response_data in function {response_data}')
        response.movecar = response_data  # 요청 성공 여부 (예시)
        return response
