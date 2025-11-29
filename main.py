#!/usr/bin/env pybricks-micropython

import threading
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

# ==========================================
# [설정 1] 컵홀더 설정 (터치 센서 버전)
# ==========================================
GRAB_SPEED = 150
OPEN_POS = 0
CLOSE_POS = 100 

# ==========================================
# [설정 2] 레벨링(밸런싱) 설정
# ==========================================
MAX_INTEGRAL = 100
MAX_OUTPUT = 720
DEADBAND = 1
LOOP_DT_MS = 10
LOOP_DT_S = LOOP_DT_MS / 1000  # 0.01s

# PID 게인 (사용자의 leveling.py 기준)
Kp = 7.0
Ki = 0.04
Kd = 0.6

# ==========================================
# [초기화] 객체 생성
# ==========================================
ev3 = EV3Brick()

# 1. 컵홀더 장치 (Motor D, TouchSensor S1)
motor_cup = Motor(Port.D)
touch_sensor = TouchSensor(Port.S1)

# 2. 레벨링 장치 (Motor A, GyroSensor S4)
motor_roll = Motor(Port.A)
gyro_roll = GyroSensor(Port.S4)


# ==========================================
# [함수 1] 컵홀더 로직 (별도의 쓰레드로 실행)
# ==========================================
def cupholder_task():
    """터치 센서 버튼을 이용한 컵홀더 기능"""
    
    # 초기화 동작
    motor_cup.reset_angle(0)
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    while True:
        # 1. 컵 감지 대기 (버튼 눌림 확인)
        while not touch_sensor.pressed():
            wait(100) # CPU 점유율을 낮추기 위한 대기
            
        # 2. 잡기 (버튼 눌림 -> 닫기)
        # wait=False: 컵 크기가 다양해도 멈추지 않음
        motor_cup.run_target(GRAB_SPEED, CLOSE_POS, then=Stop.HOLD, wait=False)

        # 3. 유지 대기 (버튼 떨어짐 확인)
        while touch_sensor.pressed():
            wait(100)
            
        # 4. 놓기 (버튼 뗌 -> 열기)
        motor_cup.stop() # 힘 풀기
        wait(50)
        motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
        
        # 재동작 방지 대기
        wait(500)


# ==========================================
# [함수 2] 레벨링 로직 (메인 루프로 실행)
# ==========================================
def leveling_task():
    """수평 유지 균형 잡기 (가장 높은 우선순위)"""
    
    # 1. 자이로 초기화
    ev3.screen.clear()
    ev3.screen.print("Init Gyro...")
    ev3.speaker.beep()
    
    wait(1000)
    gyro_roll.reset_angle(0)
    wait(500)
    
    # 바이어스(0점 오차) 측정
    ev3.screen.print("Calibrating...")
    bias = 0
    samples = 100
    total = 0
    for _ in range(samples):
        total += gyro_roll.speed()
        wait(10)
    bias = total / samples
    
    ev3.speaker.beep(frequency=800, duration=200)
    ev3.screen.print("Ready!")
    
    # 2. PID 제어 루프 시작
    integral = 0
    prev_error = 0
    target = 0
    angle_est = 0
    
    while True:
        # (1) 센서 값 읽기 & 각도 계산
        raw_rate = gyro_roll.speed()
        rate = raw_rate - bias
        angle_est += rate * LOOP_DT_S
        
        # (2) 오차 계산
        error = angle_est - target
        if abs(error) < DEADBAND:
            error = 0
            
        # (3) PID 계산
        P_term = Kp * error
        
        integral += error
        # 적분 누적 제한
        if integral > MAX_INTEGRAL: integral = MAX_INTEGRAL
        elif integral < -MAX_INTEGRAL: integral = -MAX_INTEGRAL
        I_term = Ki * integral
        
        derivative = error - prev_error
        D_term = Kd * derivative
        prev_error = error
        
        output = P_term + I_term + D_term
        
        # 출력 제한
        if output > MAX_OUTPUT: output = MAX_OUTPUT
        elif output < -MAX_OUTPUT: output = -MAX_OUTPUT
        
        # (4) 모터 실행
        motor_roll.run(-output)
        
        # (5) 주기 맞추기 (매우 중요)
        wait(LOOP_DT_MS)

# ==========================================
# [메인 실행부]
# ==========================================

# 1. 컵홀더 기능을 '일꾼(Thread)'에게 맡겨서 백그라운드에서 실행시킴
t = threading.Thread(target=cupholder_task)
t.daemon = True # 프로그램이 꺼지면 같이 꺼지도록 설정
t.start()

# 2. 메인 프로그램은 '균형 잡기'에 전념함
leveling_task()
