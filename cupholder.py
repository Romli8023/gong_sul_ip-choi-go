#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor  # UltrasonicSensor 대신 TouchSensor 사용
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

# ==== 설정 값 ====
GRAB_SPEED = 150
OPEN_POS = 0
CLOSE_POS = 100 

# ==== 초기화 ====
ev3 = EV3Brick()
motor_cup = Motor(Port.D)

# [변경됨] 초음파 센서 -> 터치 센서 (S1 포트)
touch_sensor = TouchSensor(Port.S1)

def initialize_arm():
    """시작 시 팔을 열린 위치로 초기화"""
    ev3.screen.clear()
    print("Initializing...")
    
    # 0점 조절 및 팔 열기
    motor_cup.reset_angle(0)
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    ev3.speaker.beep()
    print("Ready.")

# ==== 메인 루프 ====

initialize_arm()

while True:
    # 1. 컵 감지 대기 (버튼이 눌릴 때까지 대기)
    ev3.screen.clear()
    ev3.screen.print("Waiting for Cup...")
    
    # 버튼이 눌리지 않은 상태(False)라면 계속 기다림
    while not touch_sensor.pressed():
        wait(10) # 짧게 대기하며 감시
    
    # ------------------------------------------------
    # 이 아래로 내려왔다는 것은 버튼이 '눌렸다'는 뜻입니다.
    # ------------------------------------------------

    # 2. 팔 닫기 (잡기)
    ev3.screen.print("Grabbing!")
    
    # wait=False 중요: 컵 크기에 따라 모터가 끝까지 못 가더라도 
    # 프로그램이 멈추지 않고 계속 버튼 상태를 확인할 수 있게 함
    motor_cup.run_target(GRAB_SPEED, CLOSE_POS, then=Stop.HOLD, wait=False)

    # 3. 컵 제거 대기 (버튼이 떨어질 때까지 대기)
    ev3.screen.print("Holding...")
    
    # 버튼이 눌려 있는 상태(True)라면 계속 잡고 있음
    while touch_sensor.pressed():
        wait(10)
        
    # ------------------------------------------------
    # 이 아래로 내려왔다는 것은 버튼이 '떼어졌다'는 뜻입니다.
    # ------------------------------------------------

    # 4. 팔 열기
    ev3.screen.print("Released.")
    
    # [안전 장치] 꽉 쥐고 있던 힘 풀기
    motor_cup.stop()
    wait(50) 

    # 팔을 활짝 엽니다.
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    ev3.screen.print("Ready Next")
    # 너무 빠른 재동작 방지 (0.5초 대기)
    wait(500)