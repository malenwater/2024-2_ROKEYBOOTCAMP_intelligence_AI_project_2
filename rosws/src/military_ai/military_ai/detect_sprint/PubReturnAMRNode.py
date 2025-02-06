
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty  # null 값 처리를 위해 Empty 메시지 사용

class PubReturnAMRNode(Node):
    def __init__(self):
        super().__init__('PubReturnAMRNode')
        self.get_logger().info('PubReturnAMRNode start')
        
        self.NavGo_publisher = self.create_publisher(
            Empty,
            'nav_command',
            10
            )
        
        self.get_logger().info('PubReturnAMRNode end')

    def publish_NavGo(self):
        msg = Empty()
        self.NavGo_publisher.publish(msg)
        self.get_logger().info('Published publish_NavGo')
