
import rclpy
from rclpy.node import Node
from military_interface.msg import TrackingPos

class PubTrackingPosNode(Node):
    def __init__(self):
        super().__init__('PubTrackingPosNode')
        self.get_logger().info('PubTrackingPosNode start')
        
        self.TrackingPos_publisher = self.create_publisher(
            TrackingPos,
            'TrackingPos',
            10
            )
        self.get_logger().info('PubTrackingPosNode end')

    def publish_TrackingPos(self,send_data):
        msg = TrackingPos()
        msg.trackingpos = send_data
        self.TrackingPos_publisher.publish(msg)
        self.get_logger().info('Published publish_TrackingPos')
