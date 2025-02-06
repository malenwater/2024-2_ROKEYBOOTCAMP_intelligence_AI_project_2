
import rclpy
from rclpy.node import Node
from military_interface.msg import Return


class SubReturnAMRNode(Node):
    def __init__(self,mainServer):
        super().__init__('SubReturnAMRNode')
        self.get_logger().info(f'SubReturnAMRNode start')
        self.return_flag = False
        self.subscription = self.create_subscription(
            Return,
            'Return',
            self.return_callback,
            10
        )
        self.mainServer = mainServer
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'SubReturnAMRNode end')
            
    def return_callback(self, msg):
        self.get_logger().info(f'Received return message in SubReturnAMRNode: {msg}')
        self.mainServer.send_return()
