
import rclpy
from rclpy.node import Node
from military_interface.msg import StopCar


class StopAMRNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.return_flag = False
        self.subscription = self.create_subscription(
            StopCar,
            'StopCar',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def get_return_flag(self):
        return self.return_flag
    
    def set_return_flag(self,flag):
        self.return_flag = flag
        
    def listener_callback(self, msg):
        self.get_logger().info(f'StopAMRNode listener_callback')
        self.set_return_flag(True)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = StopAMRNode()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
