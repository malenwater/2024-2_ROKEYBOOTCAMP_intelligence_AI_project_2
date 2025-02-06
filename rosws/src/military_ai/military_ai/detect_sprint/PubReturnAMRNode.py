
import rclpy
from rclpy.node import Node
from military_interface.msg import NavGo

class PubReturnAMRNode(Node):
    def __init__(self):
        super().__init__('PubReturnAMRNode')
        self.get_logger().info('PubReturnAMRNode start')
        
        self.NavGo_publisher = self.create_publisher(
            NavGo,
            'NavGo',
            10
            )
        self.get_logger().info('PubReturnAMRNode end')

    def publish_NavGo(self):
        msg = NavGo()
        self.NavGo_publisher.publish(msg)
        self.get_logger().info('Published publish_NavGo')
