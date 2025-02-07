
import rclpy
from rclpy.node import Node
from military_interface.msg import Return

class PubReadyReturnAMRNode(Node):
    def __init__(self):
        super().__init__('PubReadyReturnAMRNode')
        self.get_logger().info('PubReadyReturnAMRNode start')
        
        self.Ready_stop_publisher = self.create_publisher(
            Return,
            'Return',
            10
            )
        
        self.get_logger().info('PubReadyReturnAMRNode end')

    def publish_Ready_stop(self):
        msg = Return()
        self.Ready_stop_publisher.publish(msg)
        self.get_logger().info('Published publish_Ready_stop')
