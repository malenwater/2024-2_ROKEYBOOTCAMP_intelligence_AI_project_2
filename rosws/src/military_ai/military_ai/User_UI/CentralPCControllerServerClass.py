import rclpy
from rclpy.node import Node
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from .PubReadyStopAMRNode import PubReadyStopAMRNode
from .PubReadyReturnAMRNode import PubReadyReturnAMRNode
from .SrvReadyObjectionSeletionAMRNode import SrvReadyObjectionSeletionAMRNode
from .SrvReadyRequestSolveAMRNode import SrvReadyRequestSolveAMRNode
class CentralPCControllerServerClass(Node):
    def __init__(self):
        super().__init__('CentralPCControllerServerClass')
        print(f'CentralPCControllerServerClass 시작!')
        self.running = False
        self.class_data = {"0" : "Car","1" : "Dummy"}
        self.PubReadyStopAMRNode = PubReadyStopAMRNode()
        self.PubReadyReturnAMRNode = PubReadyReturnAMRNode()
        self.SrvReadyObjectionSeletionAMRNode = SrvReadyObjectionSeletionAMRNode(self)
        self.SrvReadyRequestSolveAMRNode = SrvReadyRequestSolveAMRNode(self)
    def insert_user(self):
        self.running = True
        while self.running:
            user_input = input("시작, 정지, 복귀, 종료를 입력하세요. ")
            print(f'`{user_input}` 을 입력하였습니다.')
            if user_input == "종료":
                self.running = False
            elif user_input == "시작":
                future = self.SrvReadyRequestSolveAMRNode.select_request()
                service_response = future.movecar

                print(f'{service_response}로 데이터 수신')
                for i in range(int(len(service_response)/2)):
                    print(f'ID는 {service_response[2 * i]} 종류는 {self.class_data[service_response[2 * i + 1]]}')
                user_input = input("ID를 입력하세요. 만약 선택하지 않는다면 `다시`를 입력하세요 ")
                if user_input == "다시":
                    print(f'다시 시작합니다.')
                    continue
                try:
                    user_input = int(user_input)  # 숫자로 변환 시도
                except ValueError:  # 변환 실패하면 그냥 무시하고 다시 입력받음
                    print(f'숫자를 입력하세요. 다시 시작합니다.')
                    continue  
                future = self.SrvReadyObjectionSeletionAMRNode.select_request(user_input)
                print(f'선택되었습니다.')

            elif user_input == "정지":
                self.PubReadyStopAMRNode.publish_Ready_stop()
                print(f'`AMR이 추적상태에서 정지되어 대기상태가 되었습니다.')
            elif user_input == "복귀":
                self.PubReadyReturnAMRNode.publish_Ready_stop()
                print(f'`AMR이 대기상태일 경우, 보급소로 복귀합니다.')
                
    def stop(self):
        """YOLO 스레드 종료"""
        self.running = False

def main():
    rclpy.init()
    node = CentralPCControllerServerClass()
    
    detect_thread = threading.Thread(target=node.insert_user)
    detect_thread.start()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node) # FrontCarTracking 노드 추가
    executor.add_node(node.PubReadyStopAMRNode) # FrontCarTracking 노드 추가
    executor.add_node(node.PubReadyReturnAMRNode) # FrontCarTracking 노드 추가
    executor.add_node(node.SrvReadyObjectionSeletionAMRNode) # FrontCarTracking 노드 추가
    executor.add_node(node.SrvReadyRequestSolveAMRNode) # FrontCarTracking 노드 추가
    
    try:
        executor.spin()  # ROS 2 이벤트 루프 시작
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 쓰레드와 노드 종료 처리
        node.stop()
        detect_thread.join()  # 쓰레드 종료 대기
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()