#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float32MultiArray

# 전역 변수 (이전 중심 좌표와 시간 저장)
prev_center = None   # (center_x, center_y)
prev_time = None

def bounding_box_callback(msg):
    """
    AMR 중앙 제어 서버로부터 전달받은 바운딩 박스 정보에서 RC카의 중심 좌표를 이용해
    픽셀 단위의 속도를 계산
    메시지의 data는 [center_x, center_y, width, height] 순서의 값들이 들어있다고 가정.
    """
    global prev_center, prev_time

    # 메시지에서 값 추출
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

    # 현재 시간 (초 단위, rospy.Time.now()는 rospy.Time 타입이므로 to_sec() 호출)
    current_time = rospy.get_time()

    # 처음 수신하는 경우에는 이전값 초기화 후 리턴
    if prev_center is None or prev_time is None:
        prev_center = (center_x, center_y)
        prev_time = current_time
        rospy.loginfo("초기 바운딩 박스 수신: center=(%.2f, %.2f), size=(%.2f, %.2f)",
                      center_x, center_y, width, height)
        return

    # 시간 차이 (초)
    dt = current_time - prev_time
    if dt <= 0:
        rospy.logwarn("유효하지 않은 시간 차: dt=%f", dt)
        return

    # 중심 좌표 변화량 계산 (픽셀 단위)
    dx = center_x - prev_center[0]
    dy = center_y - prev_center[1]

    # 유클리드 거리 (픽셀)
    distance = math.hypot(dx, dy)

    # 속도 계산 (픽셀/초)
    speed = distance / dt

    rospy.loginfo("현재 바운딩 박스: center=(%.2f, %.2f), size=(%.2f, %.2f)", center_x, center_y, width, height)
    rospy.loginfo("Δt=%.3f sec, Δdistance=%.2f pixel, 속도=%.2f pixel/sec", dt, distance, speed)

    # 이전 값 업데이트
    prev_center = (center_x, center_y)
    prev_time = current_time

def main():
    """
    ROS 노드를 초기화하고 바운딩 박스 토픽을 구독하는 메인 함수
    """
    rospy.init_node('rc_car_speed_calculator', anonymous=True)
    rospy.loginfo("RC Car Speed Calculator 노드 시작")

    # "bounding_box" 토픽을 Float32MultiArray 타입으로 구독
    rospy.Subscriber("bounding_box", Float32MultiArray, bounding_box_callback)

    # ROS 스핀으로 콜백 대기
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
