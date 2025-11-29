#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

# 포트 설정
PITCH_MOTOR_PORT = Port.A
ROLL_MOTOR_PORT = Port.B
PITCH_GYRO_PORT = Port.S3
ROLL_GYRO_PORT = Port.S4

TARGET_ANGLE = 0 

# PID 게인 (L: 강하게 / R: 부드럽게)
KP_P, KI_P, KD_P = 2.2, 0.05, 3.0
KP_R, KI_R, KD_R = 1.8, 0.02, 2.0

MAX_INTEGRAL = 100 
DEADZONE = 8  # 오차 무시 범위 (±2도)

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
pitch_motor = Motor(PITCH_MOTOR_PORT)
roll_motor = Motor(ROLL_MOTOR_PORT)
pitch_gyro = GyroSensor(PITCH_GYRO_PORT)
roll_gyro = GyroSensor(ROLL_GYRO_PORT)

ev3.speaker.beep()

# 자이로 강제 0점 설정
force_reset_gyro(pitch_gyro)
force_reset_gyro(roll_gyro)

ev3.speaker.beep()

pid_pitch = PIDController(KP_P, KI_P, KD_P, TARGET_ANGLE)
pid_roll = PIDController(KP_R, KI_R, KD_R, TARGET_ANGLE)

# 메인 루프
while True:
    angle_p = pitch_gyro.angle()
    angle_r = roll_gyro.angle()
    
    power_p = pid_pitch.compute(pitch_gyro.angle())
    power_r = pid_roll.compute(roll_gyro.angle())
    
    pitch_motor.run(-power_p)
    roll_motor.run(-power_r)
    
    # 디버깅 출력 (각도 | 파워)
    print("P:{}|{}  R:{}|{}".format(angle_p, int(power_p), angle_r, int(power_r)))

    wait(10)
