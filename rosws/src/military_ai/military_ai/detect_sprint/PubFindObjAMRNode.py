
from rclpy.node import Node
from std_msgs.msg import Float32
class PubFindObjAMRNode(Node):
    def __init__(self):
        super().__init__('PubFindObjAMRNode')
        self.get_logger().info('PubFindObjAMRNode start')
        
        self.FindObj_publisher = self.create_publisher(
            Float32,
            'cmd_mode',
            10
            )
        self.get_logger().info('PubFindObjAMRNode end')

    def publish_float_to_AMR_cmdVel(self):
        msg = Float32()
        msg.data = 3.0
        self.FindObj_publisher.publish(msg)
        self.get_logger().info(f"Published Command Mode: {0}")
