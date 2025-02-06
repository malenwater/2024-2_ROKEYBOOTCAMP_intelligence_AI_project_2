import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import threading
import sys
import select
import termios
import tty

class StopAMRNode(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 3개의 웨이포인트 (예시 값들)
        self.way = [
            {'x': 0.1750425100326538, 'y': 0.05808566138148308, 'z': 0.0, 'orientation_z': -0.04688065682721989, 'orientation_w': 0.9989004975549108},
            {'x': 1.0, 'y': 1.0, 'z': 0.0, 'orientation_z': 0.0, 'orientation_w': 1.0},
            {'x': 2.0, 'y': 2.0, 'z': 0.0, 'orientation_z': 0.0, 'orientation_w': 1.0}
        ]

    def publish_waypoint(self, waypoint):
        # Create the pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Set the position from the waypoint
        pose_msg.pose.pose.position.x = waypoint['x']
        pose_msg.pose.pose.position.y = waypoint['y']
        pose_msg.pose.pose.position.z = waypoint['z']

        # Set the orientation (in quaternion form)
        pose_msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=waypoint['orientation_z'],
            w=waypoint['orientation_w']
        )

        # Set the covariance values (fixed values as in your original code)
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        # Publish the pose message
        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Waypoint published: {waypoint}')

    def publish_waypoints(self):
        for waypoint in self.way:
            self.publish_waypoint(waypoint)

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'p':  # 'p' 키가 눌리면 웨이포인트 발행 시작
                    node.get_logger().info('Key "p" pressed. Publishing waypoints...')
                    node.publish_waypoints()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = StopAMRNode()

    # Start the keyboard listener in a separate thread
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
