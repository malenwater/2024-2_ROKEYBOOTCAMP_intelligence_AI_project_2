import rclpy
from rclpy.node import Node
from front_car_tracking import FrontCarTracking
import threading
from rclpy.executors import MultiThreadedExecutor
import time
class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        # FrontCarTracking 노드 생성 (하위 노드)
        self.front_node = FrontCarTracking()
        self.data = None
        # YOLO 쓰레드를 백그라운드로 시작
        self.yolo_thread = threading.Thread(target=self.front_node.run_detect_controller)
        self.yolo_thread.start()
        self.data_size = [480,640]
    def run_other(self):
        while True:
            self.data = self.front_node.get_follow_car()
            if self.data != None:
                x1, y1, x2, y2, cls__ =  self.data
                print("성공 값 가져와서 넣기",self.data)
                print("좌우로 오른쪽 값만큼 ",(480 / 2) - ((x2 - x1) / 2))
                print("위아래로 오른쪽 값만큼 ",(640 / 2) - ((y2 - y1) / 2))
            time.sleep(1)

    def shutdown(self):
        # 노드 종료 시, 쓰레드도 종료하도록 처리
        self.front_node.running = False
        self.yolo_thread.join()  # 쓰레드 종료 대기


# 메인 함수
def main(args=None):
    rclpy.init(args=args)

    # TestNode 생성
    node = TestNode()
    front_node = node.front_node  # FrontCarTracking 노드 가져오기

    # MultiThreadedExecutor를 사용하여 두 개의 노드를 동시에 실행
    executor = MultiThreadedExecutor()
    executor.add_node(node)       # TestNode 추가
    executor.add_node(front_node) # FrontCarTracking 노드 추가
    
    yolo_thread = threading.Thread(target=node.run_other)
    yolo_thread.start()
    
    try:
        executor.spin()  # ROS 2 이벤트 루프 시작
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 쓰레드와 노드 종료 처리
        node.shutdown()  # 쓰레드 종료 및 노드 종료
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()