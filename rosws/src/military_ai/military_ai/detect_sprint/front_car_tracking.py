import rclpy
from rclpy.node import Node
from detect_car import YoloProcessor
import threading
import time

class FrontCarTracking(Node):
    """ROS2 서비스 노드 (YOLO 실행 요청을 처리)"""
    def __init__(self):
        super().__init__('front_car_tracking')

        # YOLO 실행 클래스를 인스턴스로 생성하여 관리
        self.yolo_processor = YoloProcessor()
        yolo_thread = threading.Thread(target=self.yolo_processor.run_yolo)
        yolo_thread.start()
        
        while True:
            detection_result = self.yolo_processor.get_result()  # ✅ 직접 변수 읽기
            if detection_result is not None:
                print("📸 YOLO 감지 결과:", detection_result)
            time.sleep(0.01)  # 메인 스레드 루프 간격 조정
            
        # 서비스 생성 (콜백 클래스를 활용)
        # self.srv = self.create_service(SetBool, 'move_service', self.service_callback)

    def service_callback(self, request, response):
        """서비스 요청이 들어오면 YOLO 실행"""
        # if request.data:  # 요청이 True일 경우 YOLO 실행
        #     self.get_logger().info('YOLO 실행 요청을 받음. 실행 시작...')
        #     response.success = tracking_result
        #     response.message = "YOLO 실행 완료"
        # else:
        #     response.success = False
        #     response.message = "요청이 올바르지 않음"

        return response
    def shutdown(self):
        """노드 종료 시 YOLO 스레드 안전하게 종료"""
        self.get_logger().info("YOLO 스레드를 종료합니다.")
        self.yolo_processor.stop()  # YOLO 프로세서 종료
        self.yolo_thread.join()  # YOLO 스레드 종료 대기

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
