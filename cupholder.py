#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

# ==== 설정 값 ====
# 모터 속도 (deg/s)
GRAB_SPEED = 100
# 열린 상태 각도
OPEN_POS = 0
# 닫힌 상태 각도 (컵을 잡는 각도)
CLOSE_POS = 60

# ==== 초기화 ====
ev3 = EV3Brick()

# 포트 설정 (D: 모터, S1: 터치센서)
motor_cup = Motor(Port.D)
ultrasonic_sensor = UltrasonicSensor(Port.S1)
#기준 거리
DISTANCE_THRESHOLD = 30

def initialize_arm():
    """시작 시 팔을 열린 위치로 초기화"""
    ev3.screen.clear()
    
    # 현재 위치를 0으로 설정
    motor_cup.reset_angle(0)
    
    # 열린 위치로 이동 (확실히 하기 위해)
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    ev3.speaker.beep()

# ==== 메인 루프 ====

# 초기화 실행
initialize_arm()

while True:
    # 1. 컵 대기 
    while not ultrasonic_sensor.distance() <= DISTANCE_THRESHOLD:
        wait(100) # 0.1초 간격 확인

    # 2. 컵 감지됨 -> 팔 닫기
    motor_cup.run_target(GRAB_SPEED, CLOSE_POS, then=Stop.HOLD, wait=True)

    # 3. 컵 제거 대기 
    while ultrasonic_sensor.distance() <= DISTANCE_THRESHOLD:
        wait(100)

    # 4. 컵 제거됨 -> 팔 열기
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    # 다음 루프 전 잠시 대기
    wait(500)
