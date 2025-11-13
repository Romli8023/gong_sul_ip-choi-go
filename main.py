#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait

MAX_INTEGRAL = 50
# --- 1. 하드웨어 및 변수 초기화 ---
ev3 = EV3Brick()
# 사용하지 않는 모터는 주석 처리하거나, 고정용으로만 사용
# motor_pitch = Motor(Port.A) 
motor_roll = Motor(Port.A)  # A 포트 모터를 'roll' 제어용으로 사용
gyro_roll = GyroSensor(Port.S4) # S4 포트 자이로 센서 사용

# PID 제어 상수 (이 값들을 조절하며 최적의 성능을 찾아야 합니다)
Kp = 1.5  # 비례 상수: 오차에 비례하여 힘을 조절 (가장 중요)
Ki = 0.01  # 적분 상수: 남아있는 미세한 오차를 없앰
Kd = 0.0  # 미분 상수: 급격한 움직임을 억제하고 안정성을 높임

# PID 계산을 위한 변수들
integral = 0
prev_error = 0
target = 0 # 목표 각도는 0도 (수평)

# --- 2. 시작 전 센서 초기화 함수 ---
def initialize_system():
    """프로그램 시작 시 센서 영점을 설정하고 모터를 준비합니다."""
    print("Initializing...")
    ev3.speaker.beep()
    gyro_roll.reset_angle(0)
    # motor_pitch.hold() # A모터가 흔들리지 않게 고정하려면 이 코드의 주석을 푸세요.
    wait(1000) # 센서가 안정될 때까지 대기
    ev3.speaker.beep(frequency=800, duration=200)
    print("System Ready.")

# --- 3. 메인 제어 루프 ---
initialize_system() # 프로그램 시작 시 초기화 실행

while True:
    # 현재 좌우 기울기 각도 읽기
    angle = gyro_roll.angle()
    

    # --- PID 제어 계산 ---
    error = angle - target
    if abs(error) < 1:
        motor_roll.stop() # stop()은 모터를 정지시키고 멈춘 상태를 유지
        output = 0

    else:
        # P 제어 계산
        integral = 0 
        deviation = error - prev_error
        
        output = Kp * error #

    
    integral += error

    if integral > MAX_INTEGRAL:
        integral = MAX_INTEGRAL
    elif integral < -MAX_INTEGRAL:
        integral = -MAX_INTEGRAL


    deviation = error - prev_error
    prev_error = error
    
    # 최종 모터 출력(속도) 계산
    output = Kp * error + Ki * integral + Kd * deviation
    
    # --- 모터 구동 ---
    # run() 메소드는 계산된 output 값을 모터의 속도로 사용합니다.
    motor_roll.run(output)

    # --- 상태 출력 (디버깅용) ---
    ev3.screen.clear()
    ev3.screen.print("Roll Angle:", angle)
    ev3.screen.print("Output:", int(output)) # 모터 출력값 확인
    ev3.screen.print("Integral:", int(integral))
    
    wait(10) # 루프 지연 시간