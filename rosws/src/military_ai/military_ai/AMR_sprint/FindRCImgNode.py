import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String, Float32
from military_interface.msg import TrackingPos

# YOLO 감지 결과를 구독하고 방향 명령을 퍼블리시하는 노드
class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')

        # YOLO 감지 결과 구독
        self.subscription_yolo = self.create_subscription(
            TrackingPos,
            '/TrackingPos',
            self.yolo_callback,
            10
        )

        # 이동 명령 퍼블리셔 (/cmd_mode) - Float32 타입으로 변경
        self.cmd_mode_publisher = self.create_publisher(Float32, '/cmd_mode', 10)

        # 화면 가로 크기 설정 (예제 값, 필요시 조정)
        self.frame_width = 640

        self.get_logger().info("YoloSubscriber Node Started. Listening to /yolo_detections...")

    def yolo_callback(self, msg):
        """YOLO 감지 결과를 받아서 이동 방향 결정 후 퍼블리시"""
        detections = {"x1":msg.trackingpos[0],
                      "y1":msg.trackingpos[1],
                      "w":msg.trackingpos[2],
                      "h":msg.trackingpos[3],
                      "flag":msg.trackingpos[4]}

        if not detections:
            self.publish_cmd_mode(3.0)  # 감지된 객체 없음 → 정지
            return

        for d in detections:
            x1, x2 = float(d['x1']), float(d['x2'])  # 바운딩 박스 좌표 (float 변환)
            object_center = (x1 + x2) / 2  # 객체 중심 좌표

            if object_center < self.frame_width * 0.3:
                self.publish_cmd_mode(0.0)  # 왼쪽 → 좌회전
                self.get_logger().info("Detected object on LEFT, turning LEFT")
            elif object_center > self.frame_width * 0.7:
                self.publish_cmd_mode(2.0)  # 오른쪽 → 우회전
                self.get_logger().info("Detected object on RIGHT, turning RIGHT")
            elif detections["flag"] == 1:
                self.publish_cmd_mode(3.0)  # 오른쪽 → 우회전
            else:
                self.publish_cmd_mode(1.0)  # 중앙 → 직진
                self.get_logger().info("Detected object in CENTER, moving FORWARD")
            return  # 첫 번째 감지된 객체만 처리하고 종료

    def publish_cmd_mode(self, mode):
        """이동 방향을 Float32 타입으로 /cmd_mode 토픽에 퍼블리시"""
        msg = Float32()
        msg.data = mode
        self.cmd_mode_publisher.publish(msg)
        self.get_logger().info(f"Published Command Mode: {mode}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
