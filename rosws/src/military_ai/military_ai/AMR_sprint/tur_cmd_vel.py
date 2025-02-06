import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ControlCMDModeNode(Node):
    def __init__(self):
        super().__init__('control_cmd_mode_node')
        
        # /cmd_mode 구독 (0: 좌회전, 2: 우회전, 1: 직진, 3: 정지)
        self.subscription = self.create_subscription(
            Float32,
            '/cmd_mode',
            self.cmd_mode_callback,
            10)
        
        # /cmd_vel 퍼블리시 (로봇 속도 명령)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("ControlCMDModeNode Started. Waiting for /cmd_mode commands...")

    def cmd_mode_callback(self, msg):
        cmd_vel_msg = Twist()

        if msg.data == 0.0:  # 좌회전
            cmd_vel_msg.angular.z = 0.25
            cmd_vel_msg.linear.x = 0.05
            self.get_logger().info("Turning Left")
        
        elif msg.data == 1.0:  # 직진
            cmd_vel_msg.linear.x = 0.1
            self.get_logger().info("Moving Forward")
        
        elif msg.data == 2.0:  # 우회전
            cmd_vel_msg.angular.z = -0.25
            cmd_vel_msg.linear.x = 0.05
            self.get_logger().info("Turning Right")
           
        elif msg.data == 3.0:  # 정지
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().info("Stopping")

        else:
            self.get_logger().warn(f"Unknown command received: {msg.data}")

        self.publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlCMDModeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
