import rclpy
from rclpy.node import Node
from detect_car import YoloProcessor
import threading
import time
from military_interface.msg import StopCar
from military_interface.srv import AskRight, MoveCar

class FrontCarTracking(Node):
    """ROS2 서비스 노드 (YOLO 실행 요청을 처리)"""
    def __init__(self):
        super().__init__('front_car_tracking')
        self.follow_car = None
        self.lock = threading.Lock()
        self.running = False
        # YOLO 실행 클래스를 인스턴스로 생성하여 관리
        self.yolo_processor = YoloProcessor()
        self.yolo_thread = threading.Thread(target=self.yolo_processor.run_yolo)
        self.yolo_thread.start()
        
        # while True:
        #     detection_result = self.yolo_processor.get_result()  # ✅ 직접 변수 읽기
        #     if detection_result is not None:
        #         print("📸 YOLO 감지 결과:", detection_result)
        #     time.sleep(0.01)  # 메인 스레드 루프 간격 조정

        self.move_car_srv = self.create_service(MoveCar, 'move_car', self.move_car_callback)
        self.get_logger().info('MoveCar Service Ready')
        
        # self.ask_right_cli = self.create_client(AskRight, 'ask_right')
        # while not self.ask_right_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for AskRight service...')
        # self.get_logger().info('AskRight service Ready')
            
        self.subscription = self.create_subscription(
            StopCar,
            'stop_car',
            self.stop_car_callback,
            10
        )
        self.get_logger().info('Subscribed to StopCar topic')
        
    def run_detect_controller(self):
        self.running = True
        while self.running:
            pass
    
    def move_car_callback(self, request, response):
        self.get_logger().info(f'Received MoveCar request: {request}')
        # --- 제대로 되고 있나 확인해서 트래킹하는 값 정하기
        response.move = True  # 요청 성공 여부 (예시)
        return response
    
    def ask_right_send_request(self, some_value):
        self.req.some_field = some_value  # 요청 데이터 설정 (예시)
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.ask_right_response_callback)

    def ask_right_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received AskRight response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            
    def stop_car_callback(self, msg):
        self.get_logger().info(f'Received StopCar message: {msg}')
          
    def shutdown(self):
        """노드 종료 시 YOLO 스레드 안전하게 종료"""
        self.get_logger().info("YOLO 스레드를 종료합니다.")
        self.yolo_processor.stop()  # YOLO 프로세서 종료
        self.yolo_thread.join()  # YOLO 스레드 종료 대기
        
    def __del__(self):
        self.shutdown()
        
def main():
    rclpy.init()
    node = FrontCarTracking()
    
    node.get_logger().info('Front Car Tracking Node 시작!')
    rclpy.spin(node)  # ROS2 서비스 요청을 처리하는 이벤트 루프
    
    try:
        rclpy.spin(node)  # ROS2 서비스 요청을 처리하는 이벤트 루프
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()  # ROS2 노드 종료 시 쓰레드 종료 처리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
