#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch

# ==== 설정 값 (컵 잡기 관련) ====
GRAB_SPEED = 150
OPEN_POS = 0
CLOSE_POS = 100 

# ==== 포트 설정 (밸런싱 관련) ====
PITCH_MOTOR_PORT = Port.A
ROLL_MOTOR_PORT = Port.B
PITCH_GYRO_PORT = Port.S3
ROLL_GYRO_PORT = Port.S4

TARGET_ANGLE = 0 

# PID 게인 (Pitch / Roll)
KP_P, KI_P, KD_P = 2.2, 0.05, 3.0
KP_R, KI_R, KD_R = 3.0, 0.02, 2.0

MAX_INTEGRAL = 100 
DEADZONE = 8  # 오차 무시 범위 (±8도)

# ==== PID 컨트롤러 클래스 ====
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
        if dt <= 0:
            dt = 0.001

        error = self.target - current_value
        
        # 데드존: 오차가 작으면 0으로 처리 (떨림 방지)
        if abs(error) <= DEADZONE:
            error = 0
            self.integral = 0
        
        # P항
        p_term = error * self.kp
        
        # I항 (와인드업 방지)
        self.integral += error * dt
        if self.integral > MAX_INTEGRAL:
            self.integral = MAX_INTEGRAL
        elif self.integral < -MAX_INTEGRAL:
            self.integral = -MAX_INTEGRAL
        i_term = self.integral * self.ki
        
        # D항
        derivative = (error - self.prev_error) / dt
        d_term = derivative * self.kd
        
        self.prev_error = error
        
        return p_term + i_term + d_term

def force_reset_gyro(gyro):
    """자이로 강제 초기화 (프리징 방지)"""
    gyro.speed()
    wait(100)
    gyro.angle()
    wait(100)
    gyro.reset_angle(0)
    wait(100)
    
    # 0점 확인, 실패 시 재귀 호출
    if abs(gyro.angle()) > 0:
        force_reset_gyro(gyro)

# ==== 초기화 ====
ev3 = EV3Brick()

# 모터
pitch_motor = Motor(PITCH_MOTOR_PORT)
roll_motor = Motor(ROLL_MOTOR_PORT)
motor_cup  = Motor(Port.D)

# 센서
pitch_gyro = GyroSensor(PITCH_GYRO_PORT)
roll_gyro  = GyroSensor(ROLL_GYRO_PORT)
touch_sensor = TouchSensor(Port.S1)

ev3.speaker.beep()

# 자이로 강제 0점 설정
force_reset_gyro(pitch_gyro)
force_reset_gyro(roll_gyro)

ev3.speaker.beep()

# PID 컨트롤러 생성
pid_pitch = PIDController(KP_P, KI_P, KD_P, TARGET_ANGLE)
pid_roll  = PIDController(KP_R, KI_R, KD_R, TARGET_ANGLE)

def update_balance():
    """피치/롤 밸런싱을 한 번 업데이트"""
    angle_p = pitch_gyro.angle()
    angle_r = roll_gyro.angle()
    
    power_p = pid_pitch.compute(angle_p)
    power_r = pid_roll.compute(angle_r)
    
    pitch_motor.run(-power_p)
    roll_motor.run(-power_r)
    
    # 디버깅 출력
    print("P:{}|{}  R:{}|{}".format(angle_p, int(power_p), angle_r, int(power_r)))
    
    # 10ms 주기
    wait(10)

def initialize_arm():
    """시작 시 팔을 열린 위치로 초기화"""
    ev3.screen.clear()
    print("Initializing...")
    
    motor_cup.reset_angle(0)
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=True)
    
    ev3.speaker.beep()
    print("Ready.")

# ==== 메인 시작 ====
initialize_arm()

# 컵 제어 루프 + 밸런싱 동시 수행
while True:
    # 1. 컵 감지 대기
    ev3.screen.clear()
    ev3.screen.print("Waiting for Cup...")
    
    # 버튼이 눌리지 않은 동안 계속 밸런싱
    while not touch_sensor.pressed():
        update_balance()
    
    # 2. 팔 닫기 (잡기)
    ev3.screen.print("Grabbing!")
    # 컵 크기에 따라 끝까지 못 가더라도, wait=False로 밸런싱 계속
    motor_cup.run_target(GRAB_SPEED, CLOSE_POS, then=Stop.HOLD, wait=False)

    # 3. 컵 제거 대기 (버튼이 떨어질 때까지)
    ev3.screen.print("Holding...")
    while touch_sensor.pressed():
        update_balance()
        
    # 4. 팔 열기
    ev3.screen.print("Released.")
    
    # [안전 장치] 꽉 쥐고 있던 힘 풀기 + 밸런싱
    motor_cup.stop()
    for _ in range(5):   # 약 50ms
        update_balance()

    # 팔을 활짝 엶 (wait=False로 두고, 각도 확인하며 대기)
    motor_cup.run_target(GRAB_SPEED, OPEN_POS, then=Stop.HOLD, wait=False)
    
    # 목표 각도에 거의 도달할 때까지 밸런싱하면서 대기
    while abs(motor_cup.angle() - OPEN_POS) > 2:
        update_balance()
    
    ev3.screen.print("Ready Next")
    
    # 너무 빠른 재동작 방지 (0.5초 대기) + 밸런싱 유지
    for _ in range(50):  # 50 * 10ms = 500ms
        update_balance()
