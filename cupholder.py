#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

# ==== 설정 값 ====
GRAB_SPEED = 150
OPEN_POS = 0
CLOSE_POS = 100 

# [수정 1] 감지 거리 기준 완화
# 초음파 센서의 최소 인식 거리가 약 4~5cm이므로, 
# 30mm로 설정하면 인식이 잘 안될 수 있습니다. 50mm로 안전하게 설정합니다.
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
        
        # 기준 거리보다 가까우면 잡기 시작
        if dist_mm <= DETECT_DIST:
            break
        wait(100) 

    # 2. 팔 닫기 (잡기)
    ev3.screen.print("Grabbing!")
    # wait=False: 컵 크기가 다양해도 멈추지 않게 함
    motor_cup.run_target(GRAB_SPEED, CLOSE_POS, then=Stop.HOLD, wait=True)

    # 3. 물체 제거 대기
    while True:
        dist_mm = us_sensor.distance()
        
        ev3.screen.clear()
        ev3.screen.print("Holding...")
        ev3.screen.print("Dist: {} cm".format(dist_mm / 10))
        
        # 물체가 충분히 멀어지면 (감지거리 + 3cm) 루프 탈출
        if dist_mm > (DETECT_DIST + 30):
            break
        wait(100)

    # 4. 팔 열기 (여기가 핵심 수정 파트입니다)
    ev3.screen.print("Released.")
    
    # [핵심 1] 꽉 쥐고 있던 모터 힘 풀기
    # 이걸 안 하면 모터가 뻑뻑해서 바로 안 열릴 수 있습니다.
    motor_cup.stop()
    wait(100) 

    # 팔 열기 명령
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    # [핵심 2] 손이 빠질 때까지 대기 (재감지 방지)
    # 센서 앞 15cm가 깨끗해질 때까지 무조건 기다립니다.
    # 손을 뺄 시간을 벌어주는 필수 코드입니다.
    ev3.screen.print("Clear Hand...")
    while us_sensor.distance() < 150:
        wait(100)
    
    ev3.screen.print("Ready Next")
    wait(500)
