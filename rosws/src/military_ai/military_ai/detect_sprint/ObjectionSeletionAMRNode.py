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


def main(args=None):
    rclpy.init(args=args)
    word_service = RequestSolveNode()
    rclpy.spin(word_service)
    word_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
