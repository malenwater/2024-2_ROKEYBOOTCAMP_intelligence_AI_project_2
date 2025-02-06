import rclpy
from rclpy.node import Node
import threading
import time
from .detect_car import YoloProcessor
from rclpy.executors import MultiThreadedExecutor
from .PubTrackingPosNode import PubTrackingPosNode
from .SubStopAMRNode import SubStopAMRNode
from .PubReturnAMRNode import PubReturnAMRNode
from .SubReturnAMRNode import SubReturnAMRNode
from .SrvRequestSolveAMRNode import SrvRequestSolveAMRNode

class CentralAMRControllerServerClass(Node):
    """ROS2 서비스 노드 (YOLO 실행 요청을 처리)"""
    def __init__(self):
        super().__init__('front_car_tracking')
        # self.AMR_STATUS = 2
        self.AMR_STATUS = 2
        self.CAR = 0
        self.follow_car = None
        self.follow_car_ID = None
        self.detection_result = None
        self.count_lost_follow_car_ID = 0
        
        # AMR_STATUS = 0 평상시 추적
        # AMR_STATUS = 1 객체 잃은 추적
        # AMR_STATUS = 2 대기 상태
        # self.frame_time = 1 / 30
        self.frame_time = 1
        self.ask_right_response = None
        self.running = False
        
        # YOLO 실행 클래스를 인스턴스로 생성하여 관리
        self.yolo_processor = YoloProcessor(self.frame_time)
        self.yolo_thread = threading.Thread(target=self.yolo_processor.run_yolo)
        self.yolo_thread.start()

        # 연관된 node 연결
        self.PubTrackingPosNode = PubTrackingPosNode()
        self.SubStopAMRNode = SubStopAMRNode(self)
        
        self.PubReturnAMRNode = PubReturnAMRNode()
        self.SubReturnAMRNode = SubReturnAMRNode(self)
        
        self.SrvRequestSolveAMRNode = SrvRequestSolveAMRNode(self)
    def run_tracking(self):
        self.running = True
        self.get_logger().info(f'run_tracking start')
        while self.running:
            self.detection_result = self.yolo_processor.get_result()
            self.get_logger().info(f'run_tracking detection_result : {self.detection_result}')
            self.get_logger().info(f'run_tracking AMR_STATUS : {self.AMR_STATUS}')
            self.get_logger().info(f'run_tracking follow_car_ID : {self.follow_car_ID}')
            
            if self.detection_result != None and len(self.detection_result) > 0:
                self.detection_result.sort(key=lambda x: x[6], reverse=True)
                self.follow_car_ID = self.detection_result[0][5]
                
            if self.AMR_STATUS == 0 and self.follow_car_ID is not None:
                self.get_logger().info(f'AMR_STATUS {self.AMR_STATUS}, will move')
                self.is_over600fps()
                self.update_current_RC()
                self.change_tracking()
 
            elif self.AMR_STATUS == 1:
                self.check_tracking()
            elif self.AMR_STATUS == 2:
                pass
            time.sleep(self.frame_time)
    def publish_stop(self):
        self.get_logger().info(f'publish_stop')
        send_data_msg = [0.,0.,0.,0.,1.]
        self.PubTrackingPosNode.publish_TrackingPos(send_data_msg)
        
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
            self.follow_car = None
            self.count_lost_follow_car_ID += 1
        self.get_logger().info(f'update_current_RC end {self.follow_car}')
            
    def change_tracking(self):
        self.get_logger().info(f'change_tracking start')
        if self.follow_car is None:
            return
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
        self.PubTrackingPosNode.publish_TrackingPos(send_data_msg)
        self.get_logger().info(f'change_tracking end')

    def send_return(self):
        self.PubReturnAMRNode.publish_NavGo()

            
    def shutdown(self):
        """노드 종료 시 YOLO 스레드 안전하게 종료"""
        self.get_logger().info("YOLO 스레드를 종료합니다.")
        self.yolo_processor.stop()  # YOLO 프로세서 종료
        self.yolo_thread.join()  # YOLO 스레드 종료 대기
        
    def stop(self):
        """YOLO 스레드 종료"""
        self.running = False
        
    def get_detection_result(self):
        return self.detection_result
    
    def get_follow_car(self):
        return self.follow_car

    def set_follow_car(self,data):
        self.follow_car = data
    
    def set_follow_car_ID(self,data):
        self.follow_car_ID = data
    
    def set_AMR_STATUS(self,data):
        self.AMR_STATUS = data
    
def main():
    rclpy.init()
    node = CentralAMRControllerServerClass()
    node.get_logger().info('Front Car Tracking Node 시작!')
    
    detect_thread = threading.Thread(target=node.run_tracking)
    detect_thread.start()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)       # TestNode 추가
    executor.add_node(node.PubTrackingPosNode) # FrontCarTracking 노드 추가
    executor.add_node(node.SrvRequestSolveAMRNode) # FrontCarTracking 노드 추가
    executor.add_node(node.SubStopAMRNode) # FrontCarTracking 노드 추가
    executor.add_node(node.SubReturnAMRNode) # FrontCarTracking 노드 추가
    executor.add_node(node.PubReturnAMRNode) # FrontCarTracking 노드 추가
    
    try:
        executor.spin()  # ROS 2 이벤트 루프 시작
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 쓰레드와 노드 종료 처리
        node.shutdown()  # ROS2 노드 종료 시 쓰레드 종료 처리
        node.destroy_node()
        node.stop()
        detect_thread.join()  # 쓰레드 종료 대기
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
