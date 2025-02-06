import rclpy
from rclpy.node import Node
from detect_car import YoloProcessor
import threading
import time
from military_interface.msg import StopCar, TrackingPos
from military_interface.srv import AskRight, MoveCar

class FrontCarTracking(Node):
    """ROS2 서비스 노드 (YOLO 실행 요청을 처리)"""
    def __init__(self):
        super().__init__('front_car_tracking')
        # self.AMR_STATUS = 2
        self.AMR_STATUS = 0
        self.CAR = 0
        # AMR_STATUS = 0 평상시 추적
        # AMR_STATUS = 1 객체 잃은 추적
        # AMR_STATUS = 2 대기 상태
        self.found_car_ID = None
        self.follow_car_ID = None
        self.follow_car = None
        self.detection_result = None
        self.count_lost_follow_car_ID = 0
        # self.frame_time = 1 / 30
        self.frame_time = 1
        self.ask_right_event = threading.Event()  # 이벤트 생성
        self.ask_right_response = None
        self.running = False
        
        # YOLO 실행 클래스를 인스턴스로 생성하여 관리
        self.yolo_processor = YoloProcessor(self.frame_time)
        self.yolo_thread = threading.Thread(target=self.yolo_processor.run_yolo)
        self.yolo_thread.start()


        self.move_car_srv = self.create_service(MoveCar, 'move_car', self.move_car_callback)
        self.get_logger().info('MoveCar Service Ready')
        
        # self.ask_right_cli = self.create_client(AskRight, 'ask_right')
        # while not self.ask_right_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for AskRight service...')
        # self.req = AskRight.Request()
        # self.get_logger().info('AskRight service Ready')
        # self.ask_right_send_request()
        
        self.subscription = self.create_subscription(
            StopCar,
            'stop_car',
            self.stop_car_callback,
            10
        )
        self.get_logger().info('Subscribed to StopCar topic')
        
        self.TrackingPos_publisher = self.create_publisher(
            TrackingPos,
            'TrackingPos',
            10
            )


    def publish_TrackingPos(self,send_data):
        msg = TrackingPos()
        msg.trackingpos = send_data
        self.TrackingPos_publisher.publish(msg)
        self.get_logger().info('Published publish_TrackingPos')

    def run_tracking(self):
        self.running = True
        self.get_logger().info(f'run_tracking start')
        while self.running:
            self.detection_result = self.yolo_processor.get_result()
            self.get_logger().info(f'run_tracking {self.detection_result}')
            
            if self.detection_result != None and len(self.detection_result) > 0:
                self.detection_result.sort(key=lambda x: x[6], reverse=True)
                self.follow_car_ID = self.detection_result[0][5]
                self.AMR_STATUS = 0
                
            if self.AMR_STATUS == 0 and self.follow_car_ID is not None:
                self.get_logger().info(f'AMR_STATUS {self.AMR_STATUS}, will move')
                self.update_current_RC()
                self.change_tracking()
 
            elif self.AMR_STATUS == 1:
                self.check_tracking()
            elif self.AMR_STATUS == 2:
                pass
            time.sleep(self.frame_time)
    
    def is_over600fps(self):
        if self.count_lost_follow_car_ID >= 600:
            self.AMR_STATUS = 1
            
    def check_tracking(self):
        # 현재 status 1인걸로 넘기자
        # if self.count_lost_follow_car_ID >= 300 and self.detection_result != None and len(self.detection_result) > 0:
        #     x1, y1, x2, y2, track_id, detect_class = self.detection_result[0]
        #     if detect_class == "Car": # 차후에 바꿈
        #         self.found_car_ID = track_id
        #         # self.follow_car_ID = track_id
        pass
                
    def update_current_RC(self):
        self.get_logger().info(f'update_current_RC start')
        check_exit_current_follow_car_ID = False
        for obj in (self.detection_result):
            x1, y1, x2, y2, track_id, class_id, confidence = obj
            if self.follow_car_ID == track_id:
                # 이부분 큐로 전환
                self.follow_car = [x1, y1, x2, y2]
                self.count_lost_follow_car_ID = 0
                check_exit_current_follow_car_ID = True
                
                break
        if check_exit_current_follow_car_ID == False:
            self.count_lost_follow_car_ID += 1
        self.get_logger().info(f'update_current_RC end {self.follow_car}')
            
    def change_tracking(self):
        self.get_logger().info(f'change_tracking start')
        send_data_msg = []
        width = self.follow_car[2] - self.follow_car[0]
        height = self.follow_car[3] - self.follow_car[1]
        mid_x = self.follow_car[0] + width / 2
        mid_y = self.follow_car[1] + height / 2
        send_data_msg.append(mid_x)
        send_data_msg.append(mid_y)
        send_data_msg.append(width)
        send_data_msg.append(height)
        send_data_msg.append(0.)
        self.publish_TrackingPos(send_data_msg)
        self.get_logger().info(f'change_tracking end')
    
    def get_follow_car(self):
        """YOLO 결과 읽기"""
        return self.follow_car
        
    def move_car_callback(self, request, response):
        self.get_logger().info(f'Received MoveCar request: {request}')
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
            
    def stop_car_callback(self, msg):
        self.get_logger().info(f'Received StopCar message: {msg} and reseted follow_car_ID')
        self.AMR_STATUS = 2
        self.follow_car_ID = None
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
    
    detect_thread = threading.Thread(target=node.run_tracking)
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
