
import rclpy
from rclpy.node import Node
from military_interface.msg import StopCar


class SubStopAMRNode(Node):
    def __init__(self,mainServer):
        super().__init__('SubStopAMRNode')
        self.get_logger().info(f'SubStopAMRNode start')
        self.return_flag = False
        self.subscription = self.create_subscription(
            StopCar,
            'stop_car',
            self.stop_car_callback,
            10
        )
        self.mainServer = mainServer
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'SubStopAMRNode end')
            
    def stop_car_callback(self, msg):
        self.get_logger().info(f'Received StopCar message in SubStopAMRNode: {msg} and reseted follow_car_ID')
        self.mainServer.set_follow_car(None)
        self.mainServer.set_follow_car_ID(None)
        self.mainServer.set_AMR_STATUS(2)
        self.mainServer.set_AMR_STATUS_2(True)
        self.mainServer.publish_stop()
