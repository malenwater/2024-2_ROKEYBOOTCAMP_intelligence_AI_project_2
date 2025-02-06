import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # FollowWaypoints 액션 클라이언트 초기화
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        
        # initialpose 토픽을 구독하는 Subscriber 추가
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)

    def initialpose_callback(self, msg):
        # initialpose에서 x, y, yaw 값을 받음
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        self.get_logger().info(f'Received initialpose: x={x}, y={y}, yaw={yaw}')

        # 받은 값으로 웨이포인트 3개 설정
        waypoints = self.create_waypoints(x, y, yaw)

        # FollowWaypoints 액션 목표 생성 및 전송
        self.send_goal(waypoints)

    def get_yaw_from_quaternion(self, quat):
        # 쿼터니언에서 yaw(방향) 값을 추출하는 함수
        siny = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny, cosy)

    def create_waypoints(self, x, y, yaw):
        # initialpose 값으로부터 3개의 웨이포인트 생성
        waypoints = []
        
        for i in range(3):
            waypoint = PoseStamped()
            waypoint.header.stamp.sec = 0
            waypoint.header.stamp.nanosec = 0
            waypoint.header.frame_id = "map"
            waypoint.pose.position.x = x + i * 0.5  # 예시로 각 웨이포인트를 0.5 단위로 이동
            waypoint.pose.position.y = y + i * 0.5
            waypoint.pose.position.z = 0.0
            
            # yaw 값을 그대로 사용하여 오리엔테이션 설정
            waypoint.pose.orientation = self.euler_to_quaternion(0, 0, yaw + i * math.pi/6)  # 예시로 yaw값에 점차 변화 추가
            
            waypoints.append(waypoint)
        
        return waypoints

    def euler_to_quaternion(self, roll, pitch, yaw):
        # 오일러 각을 쿼터니언으로 변환
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self, waypoints):
        # FollowWaypoints 액션 목표 생성 및 전송
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        # 서버 연결 대기
        self.action_client.wait_for_server()

        # 목표 전송 및 피드백 콜백 설정
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
