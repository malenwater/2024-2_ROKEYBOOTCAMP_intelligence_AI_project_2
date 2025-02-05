import cv2

from ultralytics import YOLO
cv2.namedWindow('ds')
# Load the YOLO11 model
model = YOLO("best5car_dummy.pt")

# Open the video file
# video_path = "path/to/video.mp4"
cap = cv2.VideoCapture(2)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLO11 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True)
        #results = model.track(source="https://youtu.be/LNwODJXcvt4", conf=0.3, iou=0.5, show=True)이런식가능
        
        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # if results[0].boxes is not None and results[0].boxes.data.nelement() > 0:
        #     print("\n=== Detection Results ===")
        #     for box in results[0].boxes.data.tolist():
        #         x1, y1, x2, y2, confidence, class_id, track_id = box
        #         track_id = int(track_id) if track_id is not None else -1
        #         print(f"class: {track_id}, Confidence: {class_id}, ID: {confidence:.2f}, BBox: ({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")

  

        # Display the annotated frame
        cv2.imshow("YOLO11 Tracking", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()