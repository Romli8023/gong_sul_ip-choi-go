#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# --- 1. 하드웨어 및 변수 초기화 ---
ev3 = EV3Brick()
motor_pitch = Motor(Port.A)
motor_yaw = Motor(Port.D)
gyro_pitch = GyroSensor(Port.S1)
gyro_yaw = GyroSensor(Port.S4)

# PID 제어 상수 (이 값들을 조절하며 최적의 성능을 찾아야 합니다)
# Pitch 축 (모터 A, 센서 S1)
Kp1 = 1.2  # 비례 상수: 오차에 비례하여 힘을 조절 (가장 중요)
Ki1 = 0.1  # 적분 상수: 남아있는 미세한 오차를 없앰
Kd1 = 0.5  # 미분 상수: 급격한 움직임을 억제하고 안정성을 높임

# Yaw(Roll) 축 (모터 D, 센서 S4)
Kp2 = 1.2
Ki2 = 0.1
Kd2 = 0.5

# PID 계산을 위한 변수들
integral_1, integral_2 = 0, 0
prev_error_1, prev_error_2 = 0, 0
target_1, target_2 = 0, 0 # 목표 각도는 0도 (수평)

# --- 2. 시작 전 센서 초기화 함수 ---
def initialize_sensors():
    """프로그램 시작 시 센서 영점을 설정합니다."""
    ev3.speaker.beep()
    gyro_pitch.reset_angle(0)
    gyro_yaw.reset_angle(0)
    wait(1000) # 센서가 안정될 때까지 대기
    ev3.speaker.beep(frequency=800, duration=200)

# --- 3. 메인 제어 루프 ---
initialize_sensors() # 프로그램 시작 시 초기화 실행

while True:
    # 현재 각도 읽기
    angle_1 = gyro_pitch.angle()
    angle_2 = gyro_yaw.angle()

    # --- Pitch 축 PID 제어 계산 ---
    error_1 = target_1 - angle_1
    integral_1 += error_1
    deviation_1 = error_1 - prev_error_1
    prev_error_1 = error_1
    
    # 최종 모터 출력 계산
    output_1 = Kp1 * error_1 + Ki1 * integral_1 + Kd1 * deviation_1
    
    # --- Yaw(Roll) 축 PID 제어 계산 ---
    error_2 = target_2 - angle_2
    integral_2 += error_2
    deviation_2 = error_2 - prev_error_2
    prev_error_2 = error_2

    # 최종 모터 출력 계산
    output_2 = Kp2 * error_2 + Ki2 * integral_2 + Kd2 * deviation_2

    # --- 모터 구동 ---
    # run() 메소드는 모터의 속도를 제어합니다. 계산된 output 값을 속도로 사용합니다.
    motor_pitch.run(output_1)
    motor_yaw.run(output_2)

    # --- 상태 출력 (디버깅용) ---
    ev3.screen.clear()
    ev3.screen.print("Pitch Angle:", angle_1)
    ev3.screen.print("Yaw Angle:  ", angle_2)
    
    wait(10) # 루프 지연 시간 (너무 짧으면 불안정, 길면 반응이 느림)