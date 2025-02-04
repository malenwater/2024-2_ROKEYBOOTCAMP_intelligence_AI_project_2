import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # 실제 Move 서비스 타입으로 변경 필요


class YoloProcessor:
    """YOLO 모델을 실행하는 클래스 (로직을 분리하여 관리)"""
    def __init__(self):
        pass  # 필요하면 모델 로드 등 초기화 코드 추가 가능

    def run_yolo(self):
        """YOLO 실행 함수"""
        print('YOLO 모델 실행 중...')
        
        # (여기에 YOLO 실행 로직 추가)
        
        print('YOLO 모델 실행 완료')
        return True  # 예제에서는 YOLO 실행 성공 시 True 반환


class FrontCarTracking(Node):
    """ROS2 서비스 노드 (YOLO 실행 요청을 처리)"""
    def __init__(self):
        super().__init__('front_car_tracking')

        # YOLO 실행 클래스를 인스턴스로 생성하여 관리
        self.yolo_processor = YoloProcessor()

        # 서비스 생성 (콜백 클래스를 활용)
        self.srv = self.create_service(SetBool, 'move_service', self.service_callback)

    def service_callback(self, request, response):
        """서비스 요청이 들어오면 YOLO 실행"""
        if request.data:  # 요청이 True일 경우 YOLO 실행
            self.get_logger().info('YOLO 실행 요청을 받음. 실행 시작...')
            tracking_result = self.yolo_processor.run_yolo()  # 🚀 여기서 YOLO 실행
            response.success = tracking_result
            response.message = "YOLO 실행 완료"
        else:
            response.success = False
            response.message = "요청이 올바르지 않음"

        return response


def main():
    rclpy.init()
    node = FrontCarTracking()
    
    node.get_logger().info('Front Car Tracking Node 시작!')
    rclpy.spin(node)  # ROS2 서비스 요청을 처리하는 이벤트 루프
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
