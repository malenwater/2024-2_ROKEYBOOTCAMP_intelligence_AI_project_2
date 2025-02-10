
from rclpy.node import Node
from military_interface.msg import ControlCmd
class PubFindObjAMRNode_2(Node):
    def __init__(self):
        super().__init__('PubFindObjAMRNode_2')
        self.get_logger().info('PubFindObjAMRNode_2 start')
        
        self.FindObj_publisher = self.create_publisher(
            ControlCmd,
            'cmd_mode_2',
            10
            )
        self.get_logger().info('PubFindObjAMRNode_2 end')

    def publish_float_to_AMR_cmdVel_byVector(self):
        msg = ControlCmd()
        msg.x = 0.0
        msg.z = 0.25
        self.FindObj_publisher.publish(msg)
        self.get_logger().info(f"Published PubFindObjAMRNode_2 Command publish_float_to_AMR_cmdVel_byVector {msg.vector}")
