import cv2
import numpy as np
from ultralytics import YOLO
from sort import Sort  # sort.py, hungarian.py 필요
import time
import threading
import torch

class YoloProcessor:
    """YOLO 모델을 실행하는 클래스 (로직을 분리하여 관리)"""
    def __init__(self,frame_time=1/30):
        self.model_path = "best5car_dummy.pt"
        self.frame_time = frame_time
        self.tracking = None
        self.lock = threading.Lock()
        self.running = False

    def run_yolo(self):
        """YOLO 실행 함수"""
        print('YOLO 모델 시작')
        
        model = YOLO(self.model_path)
        # cap = cv2.VideoCapture(2)
        cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)
        self.running = True
        mot_tracker = Sort()
        
        if not cap.isOpened():
            print("카메라를 열 수 없습니다.")
            return
        
        if __name__ == "__main__":
            cv2.namedWindow('YOLO + SORT')
            if torch.cuda.is_available():
                print(f"GPU 사용 가능: {torch.cuda.get_device_name(0)}")
            else:
                print("GPU 사용 불가, CPU만 사용 중입니다.")
            
        while self.running:
            ret, frame = cap.read()
            if not ret:
                break
            start_time = time.time()
            # 1) YOLO 추론
            results = model.predict(frame, conf=0.5, verbose=False) #
            yolo_processing_time = time.time() - start_time
            wait_time = self.frame_time - yolo_processing_time
            
            # 2) detections 배열과, 별도의 classes 리스트를 동시에 준비
            detections = []
            classes = []   # YOLO의 클래스 인덱스를 보관할 리스트

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    score = float(box.conf[0])
                    cls_index = int(box.cls[0])  # YOLO 클래스 인덱스
                    
                    # (x1, y1, x2, y2, score) 형태로 SORT에 넘길 검출 정보
                    detections.append([x1, y1, x2, y2, score])
                    # 클래스 인덱스를 별도로 보관
                    classes.append(cls_index)

            detections = np.array(detections)

            # 3) SORT 업데이트 → 트래킹 결과
            tracked_objects = mot_tracker.update(detections)  # Nx5: x1, y1, x2, y2, track_id
            
            # 트래킹된 객체들에 클래스 이름을 추가
            tracked_objects_with_class = []

            for i, obj in enumerate(tracked_objects):
                x1, y1, x2, y2, track_id = obj
                
                if i < len(classes):
                    class_name = model.names[classes[i]]
                else:
                    class_name = "Unknown"
                
                # 트래킹 정보와 클래스 이름을 튜플로 저장
                tracked_objects_with_class.append((x1, y1, x2, y2, track_id, class_name))

                            
            with self.lock:
                self.tracking = tracked_objects_with_class
            
            if __name__ == "__main__":
                # print(tracked_objects)
                # 4) 시각화
                if len(tracked_objects) > 0:
                    for i, obj in enumerate(tracked_objects):
                        x1, y1, x2, y2, track_id = obj
                        x1, y1, x2, y2, track_id = int(x1), int(y1), int(x2), int(y2), int(track_id)

                        # i번째 트래킹 결과 ←→ i번째 YOLO 검출의 클래스
                        # (단, 여러 객체가 있는 경우, 순서가 뒤섞일 수 있으니 주의!)
                        if i < len(classes):
                            class_name = model.names[classes[i]]
                        else:
                            class_name = "Unknown"

                        # 최종 표시: 클래스 이름 + 트랙ID
                        label = f"{class_name} (ID: {track_id})"

                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                        cv2.putText(frame, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0,255,0), 2)
                    cv2.imshow("YOLO + SORT", frame)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
            
            if wait_time > 0:
                # print(wait_time)
                time.sleep(wait_time)
            
        cap.release()
        cv2.destroyAllWindows()
        print('YOLO 모델 종료')
    def get_result(self):
        """YOLO 결과 읽기"""
        with self.lock:  # 🔒 데이터 충돌 방지
            return self.tracking

    def stop(self):
        """YOLO 스레드 종료"""
        self.running = False
        
if __name__ == "__main__":
    test = YoloProcessor(1/30)
    test.run_yolo()