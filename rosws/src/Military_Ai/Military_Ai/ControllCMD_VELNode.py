#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# 전역 변수
prev_center = None
prev_time = None
last_detect_time = None  # 마지막으로 RC카를 감지한 시간
amr_speed = 0.0  # AMR의 속도
cmd_vel_pub = None  # cmd_vel 퍼블리셔

# AMR 속도 제어 파라미터
MAX_SPEED = 0.5   # AMR 최대 속도 (m/s)
MIN_SPEED = 0.05  # AMR 최소 속도 (m/s)
STOP_TIME = 2.0  # RC카를 잃어버린 후 몇 초 후에 제자리 회전할지
ROTATE_SPEED = 0.3  # 제자리 회전 속도 (rad/s)


def bounding_box_callback(msg):
    """
    RC카의 바운딩 박스를 수신하고 AMR의 속도를 조절하는 함수.
    """
    global prev_center, prev_time, last_detect_time, amr_speed, cmd_vel_pub

    # 메시지에서 데이터 추출
    try:
        data = msg.data
        if len(data) < 4:
            rospy.logwarn("수신한 데이터 길이가 부족합니다: %s", data)
            return

        center_x = float(data[0])
        center_y = float(data[1])
        width    = float(data[2])
        height   = float(data[3])
    except Exception as e:
        rospy.logerr("데이터 파싱 중 오류 발생: %s", e)
        return

    current_time = rospy.get_time()

    # RC카를 감지한 시간 업데이트
    last_detect_time = current_time

    # 첫 프레임이면 초기화 후 종료
    if prev_center is None or prev_time is None:
        prev_center = (center_x, center_y)
        prev_time = current_time
        rospy.loginfo("초기 바운딩 박스 수신: center=(%.2f, %.2f), size=(%.2f, %.2f)",
                      center_x, center_y, width, height)
        return

    # 시간 차이 계산
    dt = current_time - prev_time
    if dt <= 0:
        rospy.logwarn("유효하지 않은 시간 차: dt=%f", dt)
        return

    # 중심 좌표 변화량 계산
    dx = center_x - prev_center[0]
    dy = center_y - prev_center[1]
    distance = math.hypot(dx, dy)  # 유클리드 거리 (픽셀)

    # 속도 계산 (픽셀/초)
    speed_pixel = distance / dt

    # RC카의 크기(바운딩 박스 높이)를 이용한 거리 추정 (가정: 높이가 클수록 가까움)
    estimated_distance = 1.0 / (height + 1e-6)  # 간단한 반비례 관계 가정

    # AMR 속도 조절 (가까울수록 감속, 멀수록 가속)
    target_speed = MAX_SPEED * estimated_distance
    target_speed = max(MIN_SPEED, min(target_speed, MAX_SPEED))  # 속도 제한

    # AMR 이동 명령 생성
    cmd_vel = Twist()
    cmd_vel.linear.x = target_speed  # 전진 속도
    cmd_vel.angular.z = 0.0  # 회전 없음

    # 속도 출력 및 퍼블리시
    rospy.loginfo("RC카 속도: %.2f pixel/sec, AMR 속도: %.2f m/s", speed_pixel, target_speed)
    cmd_vel_pub.publish(cmd_vel)

    # 이전 값 업데이트
    prev_center = (center_x, center_y)
    prev_time = current_time
    amr_speed = target_speed


def check_lost_rc_car():
    """
    RC카를 잃어버리면 AMR이 제자리에서 회전하도록 제어하는 함수.
    """
    global last_detect_time, cmd_vel_pub

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        current_time = rospy.get_time()

        if last_detect_time is not None and (current_time - last_detect_time > STOP_TIME):
            rospy.logwarn("RC카를 %d초 동안 감지하지 못함! 제자리 회전 시작", STOP_TIME)

            # 제자리 회전 명령 생성
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0  # 정지
            cmd_vel.angular.z = ROTATE_SPEED  # 회전

            # 퍼블리시
            cmd_vel_pub.publish(cmd_vel)

        rate.sleep()


def main():
    """
    ROS 노드 초기화 및 토픽 구독/퍼블리시 설정.
    """
    global cmd_vel_pub

    rospy.init_node('control_cmd_vel_node', anonymous=True)
    rospy.loginfo("Control CMD_VEL Node 시작")

    # 퍼블리셔 설정 (AMR 속도 명령)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # 바운딩 박스 구독
    rospy.Subscriber("bounding_box", Float32MultiArray, bounding_box_callback)

    # RC카 감지 여부 체크 스레드 실행
    check_lost_rc_car()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
