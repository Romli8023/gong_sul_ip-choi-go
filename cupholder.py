#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

# ==== 설정 값 ====
GRAB_SPEED = 150
OPEN_POS = 0
CLOSE_POS = 100 

# 감지 거리 기준 (mm)
# 50mm = 5cm 이내 감지 시 잡음
DETECT_DIST = 50

# ==== 초기화 ====
ev3 = EV3Brick()
motor_cup = Motor(Port.D)
us_sensor = UltrasonicSensor(Port.S1)

def initialize_arm():
    ev3.screen.clear()
    print("Initializing...")
    motor_cup.reset_angle(0)
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    ev3.speaker.beep()
    print("Ready.")

# ==== 메인 루프 ====

initialize_arm()

while True:
    # 1. 물체 감지 대기
    while True:
        dist_mm = us_sensor.distance()
        
        ev3.screen.clear()
        ev3.screen.print("Waiting...")
        ev3.screen.print("Dist: {} cm".format(dist_mm / 10))
        
        # 기준 거리(5cm)보다 가까우면 잡기 시작
        if dist_mm <= DETECT_DIST:
            break
        wait(100) 

    # 2. 팔 닫기 (잡기)
    ev3.screen.print("Grabbing!")
    # 컵 크기가 다양해도 멈추지 않도록 wait=False 유지
    motor_cup.run_target(GRAB_SPEED, CLOSE_POS, then=Stop.HOLD, wait=False)

    # 3. 물체 제거 대기 (잡은 상태 유지)
    while True:
        dist_mm = us_sensor.distance()
        
        ev3.screen.clear()
        ev3.screen.print("Holding...")
        ev3.screen.print("Dist: {} cm".format(dist_mm / 10))
        
        # 물체가 멀어지면 (감지거리 + 3cm = 8cm 이상) 루프 탈출
        if dist_mm > (DETECT_DIST + 30):
            break
        wait(100)

    # 4. 팔 열기
    ev3.screen.print("Released.")
    
    # [안전 장치] 꽉 쥐고 있던 모터 힘 풀기 (부드러운 동작을 위해 유지)
    motor_cup.stop()
    wait(100) 

    # 팔을 활짝 엽니다.
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
        
    ev3.screen.print("Ready Next")
    # 너무 빨리 재작동해서 오동작하는 것을 막기 위해 0.5초만 대기하고 바로 감시 시작
    wait(500)