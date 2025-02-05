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
        self.NOMAL = 1
        self.LOST = 2
        self.found_car_ID = None
        self.follow_car_ID = None
        self.follow_car = None
        self.detection_result = None
        self.count_lost_follow_car_ID = 0
        # self.frame_time = 1 / 30
        self.frame_time = 1
        self.lock_follow_car_ID = threading.Lock()
        self.lock_follow_car = threading.Lock()
        self.running = False
        # YOLO 실행 클래스를 인스턴스로 생성하여 관리
        self.yolo_processor = YoloProcessor(self.frame_time)
        self.yolo_thread = threading.Thread(target=self.yolo_processor.run_yolo)
        self.yolo_thread.start()


        self.move_car_srv = self.create_service(MoveCar, 'move_car', self.move_car_callback)
        self.get_logger().info('MoveCar Service Ready')
        
        self.ask_right_cli = self.create_client(AskRight, 'ask_right')
        while not self.ask_right_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for AskRight service...')
        self.req = AskRight.Request()
        self.get_logger().info('AskRight service Ready')
        self.ask_right_send_request()
        
        self.subscription = self.create_subscription(
            StopCar,
            'stop_car',
            self.stop_car_callback,
            10
        )
        self.get_logger().info('Subscribed to StopCar topic')
        
    def run_detect_controller(self):
        self.running = True
        self.get_logger().info(f'run_detect_controller start')
        while self.running:
            self.detection_result = self.yolo_processor.get_result()
            if self.follow_car_ID is not None:
               with self.lock_follow_car_ID:  # 변수를 읽을 때도 Lock을 사용하여 안전하게
                    print("Tracking Car ID:", self.detection_result)
                    
                    check_exit_current_follow_car_ID = False
                    for obj in (self.detection_result):
                        x1, y1, x2, y2, track_id, detect_class = obj
                        if self.follow_car_ID == track_id:
                            with self.lock_follow_car:
                                self.follow_car = [x1, y1, x2, y2, self.NOMAL]
                            self.count_lost_follow_car_ID = 0
                            check_exit_current_follow_car_ID = True
                            break
                        
                    if check_exit_current_follow_car_ID == False:
                        self.count_lost_follow_car_ID += 1
                            
                # print("YOLO 감지 결과:", self.detection_result)
            time.sleep(self.frame_time)
    def get_follow_car(self):
        """YOLO 결과 읽기"""
        with self.lock_follow_car:  # 🔒 데이터 충돌 방지
            return self.follow_car
        
    def move_car_callback(self, request, response):
        self.get_logger().info(f'Received MoveCar request: {request}')
        # --- 제대로 되고 있나 확인해서 트래킹하는 값 정하기
        response.move = True  # 요청 성공 여부 (예시)
        print(self.detection_result)
        if self.detection_result != None and len(self.detection_result) > 0:
            x1, y1, x2, y2, track_id, detect_class = self.detection_result[0]
            if detect_class == "Car": # 차후에 바꿈
                with self.lock_follow_car_ID:
                    self.follow_car_ID = track_id
                    response.move = True
                    
                    self.found_car_ID = track_id
        self.get_logger().info(self.yolo_processor.get_result_img_base64(self.found_car_ID))
                    
        return response
    
    def ask_right_send_request(self):
        self.req.data = ["hihi"]  # 요청 데이터 설정 (예시)
        future = self.ask_right_cli.call_async(self.req)
        future.add_done_callback(self.ask_right_response_callback)

    def ask_right_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received AskRight response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            
    def stop_car_callback(self, msg):
        self.get_logger().info(f'Received StopCar message: {msg} and reseted follow_car_ID')
        with self.lock_follow_car_ID:
            self.follow_car_ID = None
        with self.lock_follow_car:
            self.follow_car = None
            
    def shutdown(self):
        """노드 종료 시 YOLO 스레드 안전하게 종료"""
        self.get_logger().info("YOLO 스레드를 종료합니다.")
        self.yolo_processor.stop()  # YOLO 프로세서 종료
        self.yolo_thread.join()  # YOLO 스레드 종료 대기
        
    def stop(self):
        """YOLO 스레드 종료"""
        self.running = False
        
    def __del__(self):
        self.shutdown()
        
def main():
    rclpy.init()
    node = FrontCarTracking()
    node.get_logger().info('Front Car Tracking Node 시작!')
    
    detect_thread = threading.Thread(target=node.run_detect_controller)
    detect_thread.start()
    
    try:
        rclpy.spin(node)  # ROS2 서비스 요청을 처리하는 이벤트 루프
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt가 발생했습니다.')
    finally:
        node.shutdown()  # ROS2 노드 종료 시 쓰레드 종료 처리
        node.destroy_node()
        node.stop()
        detect_thread.join()  # 쓰레드 종료 대기
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
