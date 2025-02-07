# FindRCImgNode_2_test.py (로컬 테스트용)
from FindRCImgNode_2 import YoloControlNode
from military_interface.msg import TrackingPos

def test_yolo_control():
    node = YoloControlNode()

    # 가상의 YOLO 감지 결과 메시지 생성
    test_msg = TrackingPos()
    test_msg.trackingpos = [320.0, 240.0, 100.0, 100.0, 0]  # (x_center, y_center, width, height, flag)

    print("=== 테스트 시작 ===")
    node.yolo_callback(test_msg)
    print("=== 테스트 종료 ===")

if __name__ == "__main__":
    test_yolo_control()
