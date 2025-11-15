#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# ==== 상수 ====
MAX_INTEGRAL = 100
MAX_OUTPUT = 720
DEADBAND = 1

LOOP_DT_MS = 10
LOOP_DT_S = LOOP_DT_MS / 1000  # 0.01초

# PID 게인 (기본값)
Kp = 7.0
Ki = 0.04   # 초반에는 0으로 두고 안정되면 조금씩 증가
Kd = 0.6

ev3 = EV3Brick()
motor_roll = Motor(Port.A)
gyro_roll = GyroSensor(Port.S4)

integral = 0
prev_error = 0
target = 0

def init_gyro_and_bias():
    ev3.screen.clear()
    ev3.screen.print("Init gyro...")
    ev3.speaker.beep()

    # 바닥에 고정하고 센서를 건드리지 않기
    wait(1000)
    gyro_roll.reset_angle(0)
    wait(500)

    # 자이로 속도의 바이어스 측정
    ev3.screen.clear()
    ev3.screen.print("Measuring bias")
    bias = 0
    samples = 100
    total = 0
    for _ in range(samples):
        total += gyro_roll.speed()
        wait(10)
    bias = total / samples

    ev3.screen.clear()
    ev3.screen.print("Bias:", int(bias))
    ev3.speaker.beep(frequency=800, duration=200)

    return bias

# ==== 초기화 ====
bias = init_gyro_and_bias()

# 속도를 적분하여 추정 각도 계산
angle_est = 0

while True:
    # 1. 자이로 회전 속도 읽기
    raw_rate = gyro_roll.speed()
    rate = raw_rate - bias     # 바이어스 제거

    # 2. 각도 추정 (적분)
    angle_est += rate * LOOP_DT_S

    # 3. 오차 계산
    error = angle_est - target

    if abs(error) < DEADBAND:
        error = 0

    # 4. PID 계산
    P_term = Kp * error

    integral += error
    if integral > MAX_INTEGRAL:
        integral = MAX_INTEGRAL
    elif integral < -MAX_INTEGRAL:
        integral = -MAX_INTEGRAL
    I_term = Ki * integral

    derivative = error - prev_error
    D_term = Kd * derivative
    prev_error = error

    output = P_term + I_term + D_term

    if output > MAX_OUTPUT:
        output = MAX_OUTPUT
    elif output < -MAX_OUTPUT:
        output = -MAX_OUTPUT

    motor_roll.run(-output)

    # 디버그 출력
    ev3.screen.clear()
    ev3.screen.print("Rate:", int(rate))
    ev3.screen.print("Ang*:", int(angle_est))
    ev3.screen.print("Err:", int(error))
    ev3.screen.print("Out:", int(output))
    ev3.screen.print("Bias:", int(bias))

    wait(LOOP_DT_MS)