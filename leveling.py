#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

# 포트 설정
LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.B
LEFT_GYRO_PORT = Port.S3
RIGHT_GYRO_PORT = Port.S4

TARGET_ANGLE = 0 

# PID 게인 (L: 강하게 / R: 부드럽게)
KP_L, KI_L, KD_L = 1.0, 0.05, 2.0
KP_R, KI_R, KD_R = 0.5, 0.0, 0.1

MAX_INTEGRAL = 100 
DEADZONE = 2  # 오차 무시 범위 (±2도)

class PIDController:
    def __init__(self, kp, ki, kd, target):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.prev_error = 0
        self.integral = 0
        self.watch = StopWatch()
        self.watch.reset()

    def compute(self, current_value):
        dt = self.watch.time() / 1000.0 
        self.watch.reset()
        if dt <= 0: dt = 0.001

        error = self.target - current_value
        
        # 데드존: 오차가 작으면 0으로 처리 (떨림 방지)
        if abs(error) <= DEADZONE:
            error = 0
            self.integral = 0
        
        # P항
        p_term = error * self.kp
        
        # I항 (와인드업 방지)
        self.integral += error * dt
        if self.integral > MAX_INTEGRAL: self.integral = MAX_INTEGRAL
        elif self.integral < -MAX_INTEGRAL: self.integral = -MAX_INTEGRAL
        i_term = self.integral * self.ki
        
        # D항
        derivative = (error - self.prev_error) / dt
        d_term = derivative * self.kd
        
        self.prev_error = error
        
        return p_term + i_term + d_term

def force_reset_gyro(gyro):
    # 모드 변경을 통한 강제 초기화 (프리징 해결)
    gyro.speed()
    wait(100)
    gyro.angle()
    wait(100)
    gyro.reset_angle(0)
    wait(100)
    
    # 0점 확인, 실패 시 재귀 호출
    if abs(gyro.angle()) > 0:
        force_reset_gyro(gyro)

# 초기화
ev3 = EV3Brick()
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)
left_gyro = GyroSensor(LEFT_GYRO_PORT)
right_gyro = GyroSensor(RIGHT_GYRO_PORT)

ev3.speaker.beep()

# 자이로 강제 0점 설정
force_reset_gyro(left_gyro)
force_reset_gyro(right_gyro)

ev3.speaker.beep()

pid_left = PIDController(KP_L, KI_L, KD_L, TARGET_ANGLE)
pid_right = PIDController(KP_R, KI_R, KD_R, TARGET_ANGLE)

# 메인 루프
while True:
    angle_l = left_gyro.angle()
    angle_r = right_gyro.angle()
    
    power_l = pid_left.compute(left_gyro.angle())
    power_r = pid_right.compute(right_gyro.angle())
    
    left_motor.run(power_l)
    right_motor.run(power_r)
    
    # 디버깅 출력 (각도 | 파워)
    print("L:{}|{}  R:{}|{}".format(angle_l, int(power_l), angle_r, int(power_r)))

    wait(10)