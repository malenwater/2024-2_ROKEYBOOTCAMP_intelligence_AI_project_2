import cv2
import torch
import time
from ultralytics import YOLO

#노트북실험때만
# cv2.namedWindow('dsa')
 
# Load the YOLO model 경로 
model = YOLO("/home/rokey/Downloads/best5car_dummy.pt")
device = "cuda" if torch.cuda.is_available() else "cpu"

# Open the video file
cap = cv2.VideoCapture(0)

# Set camera properties for better performance
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

last_print_time = time.time()

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if not success:
        break  # 종료

    # Run YOLO tracking on the frame
    results = model.track(frame, persist=False, device=device)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    if results[0].boxes is not None and results[0].boxes.data.nelement() > 0:
        current_time = time.time()
        if current_time - last_print_time > 0.5:  # 0.5초 간격으로 출력
            print("\n=== Detection Results ===")
            for box in results[0].boxes.data.tolist():
                if len(box) >= 6:
                    x1, y1, x2, y2, confidence, class_id = box[:6]
                    track_id = int(box[6]) if len(box) > 6 and isinstance(box[6], (int, float)) else -1
                    print(f"class: {track_id}, Confidence: {class_id}, ID: {confidence:.2f}, BBox: ({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")
            last_print_time = current_time

    # Display the annotated frame
    #노트북실험때만
    # cv2.imshow("YOLO Tracking", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
