import cv2
import torch
import time
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

class YoloProcessor:
    def __init__(self, frame_time=1/30, model_path="best5car_dummy.pt"):
        """YOLO 기반 차량 감지 및 추적 프로세서"""
        package_name = "military_ai"  # ROS 2 패키지명
        package_share_directory = get_package_share_directory(package_name)
        # 모델 파일이 있는 경로 지정
        model_path = os.path.join(package_share_directory, "resource", model_path)
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(0)
        # self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.running = False
        self.frame_time = frame_time
        self.detection_result = None
        self.high_car_result = None
        
    def run_yolo(self):
        """YOLO 실행 및 객체 감지"""
        self.running = True
        last_print_time = time.time()
        
        while self.running and self.cap.isOpened():
            success, frame = self.cap.read()
            if not success:
                break
            
            results = self.model.track(frame, persist=False, device=self.device, verbose=False)
            
            if results[0].boxes is not None and results[0].boxes.data.nelement() > 0:
                current_time = time.time()
                detections = []
                high_car = []
                for box in results[0].boxes.data.tolist():
                    if len(box) >= 6:
                        x1, y1, x2, y2, track_id, confidence = box[:6]
                        class_id = int(box[6]) if len(box) > 6 and isinstance(box[6], (int, float)) else -1
                        if class_id == 0 and confidence >= 0.8:
                            high_car.append([x1, y1, x2, y2, track_id, class_id, confidence])
                        detections.append([x1, y1, x2, y2, track_id, class_id, confidence])
                self.detection_result = detections
                self.high_car_result = high_car
                
                # if current_time - last_print_time > 0.5:
                #     print("\n=== Detection Results ===")
                #     for d in self.detection_result:
                #         x1, y1, x2, y2, track_id, class_id, confidence = d
                #         print(f"class_id : {class_id} ,track_id: {track_id}, confidence: {confidence}, BBox: ({d[0]:.1f}, {d[1]:.1f}, {d[2]:.1f}, {d[3]:.1f})")
                #     last_print_time = current_time
            
            time.sleep(self.frame_time)
        
    def get_result(self):
        """최신 YOLO 감지 결과 반환"""
        # return self.detection_result
        return self.high_car
    
    def stop(self):
        """YOLO 실행 중지"""
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main = YoloProcessor()
    main.run_yolo()
