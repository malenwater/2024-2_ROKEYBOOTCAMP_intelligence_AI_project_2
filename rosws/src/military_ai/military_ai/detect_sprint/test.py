import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # move_car 서비스 요청을 위해 사용
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
import time
import threading
import torch

class YoloProcessor:
    """YOLO 모델 실행 클래스"""
    def __init__(self, frame_time=1/30):
        self.model_path = "best5car_dummy.pt"  # YOLO 모델 경로
        self.frame_time = frame_time
        self.tracking = None
        self.tracking_img = None
        self.lock_tracking = threading.Lock()
        self.running = True
        self.missing_frames = 0  # 감지되지 않은 프레임 카운트
        self.last_tracked_car = None  # 기능 4: 마지막 탐지된 차량 정보 저장

    def run_yolo(self):
        """YOLO 실행 함수"""
        print('YOLO 모델 시작 (카메라: /dev/video0)')
        model = YOLO(self.model_path)

        if torch.cuda.is_available():
            print(f"GPU 사용 가능: {torch.cuda.get_device_name(0)}")
        else:
            print("GPU 사용 불가, CPU만 사용 중입니다.")

        model.to("cpu")  # ✅ 실행을 CPU에서 강제

        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(3, 640)
        cap.set(4, 480)
        self.running = True

        if not cap.isOpened():
            print("카메라를 열 수 없습니다.")
            return

        cv2.namedWindow('YOLO Tracking')

        while self.running:
            ret, frame = cap.read()
            if not ret:
                break

            start_time = time.time()
            results = model(frame, stream=False)

            high_confidence_cars = []
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    track_id = int(box.id[0]) if box.id is not None else -1
                    cls_index = int(box.cls[0])
                    class_name = model.names[cls_index] if cls_index in model.names else "Unknown"
                    confidence = float(box.conf[0])

                    if class_name == "Car" and confidence >= 0.8:
                        high_confidence_cars.append((x1, y1, x2, y2, track_id, class_name, confidence))

            with self.lock_tracking:
                if high_confidence_cars:
                    self.tracking = high_confidence_cars
                    self.tracking_img = frame
                    self.missing_frames = 0
                    self.last_tracked_car = high_confidence_cars[0]  # 기능 4: 마지막 탐지된 차량 정보 저장
                else:
                    self.tracking = None
                    self.missing_frames += 1

            # ✅ 기능 4: 차량이 사라졌을 때 동일 차량인지 판별
            if self.missing_frames > 3 and self.last_tracked_car:
                print("🚨 차량이 사라짐, 마지막 탐지된 차량과 비교 중...")
                # TODO: 유사성 판별 로직 추가 (현재는 로그만 출력)

            for car in high_confidence_cars:
                x1, y1, x2, y2, _, _, conf = car
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Car {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("YOLO Tracking", frame)
            cv2.waitKey(1)

            yolo_processing_time = time.time() - start_time
            wait_time = self.frame_time - yolo_processing_time
            if wait_time > 0:
                time.sleep(wait_time)

        cap.release()
        cv2.destroyAllWindows()
        print('YOLO 모델 종료')

    def get_result(self):
        with self.lock_tracking:
            return self.tracking

    def stop(self):
        self.running = False


class YoloNode(Node):
    """ROS2 노드 클래스 (YOLO 실행)"""
    def __init__(self):
        super().__init__('yolo_processor')
        self.yolo_processor = YoloProcessor()
        self.publisher_ = self.create_publisher(String, '/yolo_tracking', 10)

        self.ready_status = None  # 기능 3: move_car 응답 상태 저장

        # ✅ move_car 서비스 클라이언트 설정
        self.move_car_client = self.create_client(Trigger, 'move_car')
        while not self.move_car_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ move_car 서비스 대기 중...')

        self.yolo_thread = threading.Thread(target=self.run_tracking)
        self.yolo_thread.start()

        self.get_logger().info("✅ YOLO Node Running...")

    def check_move_car(self):
        """move_car 서비스 호출하여 이동 가능 여부 확인"""
        req = Trigger.Request()
        future = self.move_car_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.ready_status = True  # 🚗 이동 가능
            else:
                self.ready_status = None  # ⏳ 대기 상태
        else:
            self.ready_status = None  # 응답 없음

    def run_tracking(self):
        """YOLO 실행 및 move_car 체크"""
        self.yolo_processor.run_yolo()
        while rclpy.ok():
            tracking_result = self.yolo_processor.get_result()

            # ✅ 기능 3: move_car 상태 확인
            self.check_move_car()
            if self.ready_status is True:
                self.get_logger().info("🚗 앞 차량 이동 가능!")
            elif self.ready_status is None:
                self.get_logger().warn("⏳ 앞 차량 대기 중...")

            # ✅ 기능 4: 차량이 사라졌을 때 동일 차량인지 판별
            if not tracking_result and self.yolo_processor.last_tracked_car:
                self.get_logger().warn("🚨 차량이 사라짐, 마지막 탐지 차량과 비교 필요!")

            time.sleep(0.1)

    def stop(self):
        self.yolo_processor.stop()
        self.yolo_thread.join()


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()