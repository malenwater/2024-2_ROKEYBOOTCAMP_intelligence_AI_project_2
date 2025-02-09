
import rclpy
from rclpy.node import Node
from military_interface.msg import StopCar

class PubReadyStopAMRNode(Node):
    def __init__(self):
        super().__init__('PubReadyStopAMRNode')
        self.get_logger().info('PubReadyStopAMRNode start')
        
        self.Ready_stop_publisher = self.create_publisher(
            StopCar,
            'stop_car',
            10
            )
        
        self.get_logger().info('PubReadyStopAMRNode end')

    def publish_Ready_stop(self):
        msg = StopCar()
        self.Ready_stop_publisher.publish(msg)
        self.get_logger().info('Published publish_Ready_stop')
