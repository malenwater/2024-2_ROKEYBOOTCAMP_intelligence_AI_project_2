from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # null 값 처리를 위해 Empty 메시지 사용

class PubSendObjNode(Node):
    def __init__(self):
        super().__init__('PubSendObjNode')
        self.get_logger().info('PubSendObjNode start')
        
        self.send_publisher = self.create_publisher(
            Float32MultiArray,
            'alram',
            10
            )
        
        self.get_logger().info('PubSendObjNode end')

    def publish_Float32MultiArray_PubSendObjNode(self):
        msg = Float32MultiArray()
        msg.data
        self.send_publisher.publish(msg)
        self.get_logger().info('Published publish_Float32MultiArray_PubSendObjNode')
