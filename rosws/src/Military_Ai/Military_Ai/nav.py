import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Empty  # null 값 처리를 위해 Empty 메시지 사용
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
from datetime import datetime

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        # 🔥 액션 클라이언트 (NavigateToPose 사용)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 🔥 구독자 추가 (이벤트 감지용)
        self.subscription = self.create_subscription(
            Empty, '/nav_command', self.nav_command_callback, 10
        )

        # 🔥 7개 좌표 설정 (고정점)
        self.goals = [
            {'x': -1.64, 'y': -0.84, 'z': 0.7, 'w': 0.7},  
            {'x': -1.64, 'y': -0.15, 'z': 0, 'w': 1.0},  
            {'x': -0.65, 'y': -0.03, 'z': -0.7, 'w': 0.7},  
            {'x': -0.81, 'y': -0.66, 'z': 0.00, 'w': 1.0},    
            {'x': 0.17, 'y': -0.66, 'z': 0.7, 'w': 0.7},      
            {'x': 0.20, 'y': 0.02, 'z': -1, 'w': 0},      
            {'x': -0.24, 'y': -0.01, 'z': 0.0, 'w': 1.0}    
        ]

        self.current_goal_index = 0
        self.goal_received = False

    def nav_command_callback(self, msg):
        """🔥 `/nav_command`에서 `null` 값을 받으면 네비게이션 시작"""
        self.get_logger().info('🚀 네비게이션 명령 수신! 목표 좌표 전송 시작')
        self.goal_received = True
        self.current_goal_index = 0
        self.send_next_goal()

    def send_next_goal(self):
        """🔥 목표 지점으로 이동하는 액션 실행"""
        if self.current_goal_index >= len(self.goals) - 1:
            self.get_logger().info('✅ 모든 목표 지점 도달 완료!')
            return

        goal = self.goals[self.current_goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.orientation.z = goal['z']
        goal_msg.pose.pose.orientation.w = goal['w']

        # 🔥 현재 시간
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # 🔥 현재 진행 중인 네비게이션 횟수
        current_navigation = f"{self.current_goal_index + 1}/6"

        # 🔥 로그 출력 (위치는 피드백에서 받음)
        self.get_logger().info(f"🕒 현재 시간: {current_time}")
        self.get_logger().info(f"🚀 네비게이션 {current_navigation} 진행 중... 목표: x={goal['x']}, y={goal['y']}")

        # 🔥 액션 실행
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """🔥 액션 서버에서 응답을 받으면 다음 목표로 이동"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ 목표가 거부됨!')
            return

        self.get_logger().info('🚀 목표가 수락됨! 이동 시작...')
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """🔥 목표 도달 후 다음 좌표로 이동"""
        result = future.result().result
        if result == 0:  # SUCCESS (성공)
            self.get_logger().info(f'🏁 목표 지점 {self.current_goal_index + 1} 도착 완료!')
            self.current_goal_index += 1
            time.sleep(0.5)  # 2초 대기 후 다음 목표 전송
            self.send_next_goal()
        else:
            self.get_logger().warn(f'❌ 목표 {self.current_goal_index + 1} 도달 실패!')

def main(args=None):
    rclpy.init(args=args)
    navigator = NavigationClient()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
