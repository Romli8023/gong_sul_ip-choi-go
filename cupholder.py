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
CLOSE_POS = 100

# 감지 거리 기준 (mm 단위)
# 100mm = 10cm 이내에 물체가 들어오면 잡습니다.
DETECT_DIST = 100 

# ==== 초기화 ====
ev3 = EV3Brick()

# 포트 설정
motor_cup = Motor(Port.D)

# [수정됨] 터치 센서 -> 초음파 센서 (S1 포트에 연결한다고 가정)
# 만약 다른 포트(예: S4)에 연결했다면 Port.S4로 바꿔주세요.
us_sensor = UltrasonicSensor(Port.S1)

def initialize_arm():
    """시작 시 팔을 열린 위치로 초기화"""
    ev3.screen.clear()
    print("Initializing...")
    
    motor_cup.reset_angle(0)
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    ev3.speaker.beep()
    print("Ready.")

# ==== 메인 루프 ====

initialize_arm()

while True:
    # 1. 물체 감지 대기 (거리가 멀면 계속 대기)
    # 화면에 현재 거리를 계속 출력합니다.
    while True:
        # 거리 측정 (mm 단위)
        dist_mm = us_sensor.distance()
        
        # 화면 출력 (mm를 cm로 변환하여 표시)
        ev3.screen.clear()
        ev3.screen.print("Waiting...")
        # {} 안에 거리 값을 넣어서 출력
        ev3.screen.print("Dist: {} cm".format(dist_mm / 10))
        
        # 물체가 기준 거리보다 가까워지면 루프 탈출 (잡기 시작)
        if dist_mm < DETECT_DIST:
            break
        
        wait(100) # 0.1초마다 갱신

    # 2. 물체 감지됨 -> 팔 닫기
    # wait=False로 설정하여 모터가 움직이는 동안에도 계속 감시하도록 함
    ev3.screen.print("Grabbing!")
    motor_cup.run_target(GRAB_SPEED, CLOSE_POS, then=Stop.HOLD, wait=True)

    # 3. 물체 제거 대기 (거리가 가까우면 계속 잡고 있음)
    while True:
        dist_mm = us_sensor.distance()
        
        ev3.screen.clear()
        ev3.screen.print("Holding Object")
        ev3.screen.print("Dist: {} cm".format(dist_mm / 10))
        
        # 물체가 사라지면 (거리가 기준보다 멀어지면) 루프 탈출
        # 약간의 여유(50mm)를 둬서 경계선에서 팔이 떨리는 것을 방지
        if dist_mm > (DETECT_DIST + 50):
            break
            
        wait(100)

    # 4. 물체 제거됨 -> 팔 열기
    ev3.screen.print("Released.")
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    # 다음 동작 전 잠시 대기
    wait(500)
