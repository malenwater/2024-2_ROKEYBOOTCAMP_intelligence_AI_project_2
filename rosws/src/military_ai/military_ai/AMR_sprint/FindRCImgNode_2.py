import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from military_interface.msg import ControlCmd  # 새로 만든 메시지 타입
from military_interface.msg import TrackingPos  # YOLO 감지 결과 메시지

class YoloControlNode(Node):
    def __init__(self):
        super().__init__('yolo_control_node')

        # YOLO 감지 결과 구독
        self.subscription_yolo = self.create_subscription(
            TrackingPos,
            '/TrackingPos',
            self.yolo_callback,
            10
        )

        # 이동 명령 퍼블리셔 (새로운 메시지 타입 사용)
        self.cmd_publisher = self.create_publisher(ControlCmd, '/cmd_vel_control', 10)

        # 기준값 설정
        self.frame_width = 640
        self.frame_height = 480
        self.ref_area = 276.0 * 235.0  # 기준 면적
        self.get_logger().info("YoloControlNode Started. Listening to /TrackingPos...")

    def yolo_callback(self, msg):
        """YOLO 감지 결과를 받아서 회전 및 속도를 계산 후 퍼블리시"""
        x_center = float(msg.trackingpos[0])
        y_center = float(msg.trackingpos[1])
        width = float(msg.trackingpos[2])
        height = float(msg.trackingpos[3])
        flag = msg.trackingpos[4]

        # flag가 1이면 정지
        if flag == 1:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("Flag detected as 1, stopping robot.")
            return

        # 객체 중심 좌표 기준 회전 값 계산 (-0.05 ~ 0.05 범위)
        rotation = (x_center - (self.frame_width / 2)) / (self.frame_width / 2) * -0.05
        rotation = max(-0.05, min(0.05, rotation))  # 범위 제한

        # 객체 면적 기준 속도 값 계산 (0.05 ~ -0.05 범위, 가까우면 후진)
        object_area = width * height
        speed = (self.ref_area - object_area) / self.ref_area * 0.05
        speed = max(-0.05, min(0.05, speed))  # 범위 제한

        self.publish_cmd(speed, rotation)
        self.get_logger().info(f"Published: speed={speed:.3f}, rotation={rotation:.3f}")

    def publish_cmd(self, speed, rotation):
        """이동 방향을 ControlCmd 메시지로 퍼블리시"""
        msg = ControlCmd()
        msg.linear_x = speed
        msg.angular_z = rotation
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
