#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait, StopWatch

# 설정 (하드웨어 포트 확인)
# 모터 포트
LEFT_MOTOR_PORT = Port.B
RIGHT_MOTOR_PORT = Port.C

# 자이로 센서 포트 (2개 사용)
LEFT_GYRO_PORT = Port.S1
RIGHT_GYRO_PORT = Port.S2

# 기본 속도 및 목표 각도
BASE_SPEED = 200     # 기본 주행 속도
TARGET_ANGLE = 0     # 목표 각도 (0도 유지 = 직진)

# PID 게인 값 (좌우 독립적으로 튜닝 가능하지만, 보통 같게 시작)
KP = 2.0
KI = 0.0
KD = 1.0

# PID 클래스 
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

        # 오차 계산
        error = self.target - current_value
        
        # P, I, D 계산
        p_term = error * self.kp
        
        self.integral += error * dt
        i_term = self.integral * self.ki
        
        derivative = (error - self.prev_error) / dt
        d_term = derivative * self.kd
        
        self.prev_error = error
        
        return p_term + i_term + d_term

# 초기화
ev3 = EV3Brick()

# 모터 초기화
left_motor = Motor(LEFT_MOTOR_PORT)
right_motor = Motor(RIGHT_MOTOR_PORT)

# 자이로 센서 2개 초기화
# 주의: 자이로 센서는 초기화 시 절대 움직이면 안 됩니다.
ev3.speaker.beep() 
print("Calibrating Gyros... Do not move")

left_gyro = GyroSensor(LEFT_GYRO_PORT)
right_gyro = GyroSensor(RIGHT_GYRO_PORT)

wait(1000) # 안정화 대기

# 각도 0점으로 리셋
left_gyro.reset_angle(0)
right_gyro.reset_angle(0)

# 각각의 PID 생성
pid_left = PIDController(KP, KI, KD, TARGET_ANGLE)
pid_right = PIDController(KP, KI, KD, TARGET_ANGLE)

ev3.speaker.beep()
print("Go!")

# 4. 메인 루프 (듀얼 센서 피드백)
while True:
    # 1. 각 센서값 읽기
    # 왼쪽 센서와 오른쪽 센서가 읽는 값이 미세하게 다를 수 있습니다.
    angle_l = left_gyro.angle()
    angle_r = right_gyro.angle()
    
    # 2. 각각 PID 계산
    # PID 출력값은 "목표 각도로 가기 위해 필요한 힘의 보정치"입니다.
    correction_l = pid_left.compute(angle_l)
    correction_r = pid_right.compute(angle_r)
    
    # 3. 모터 출력 적용 (로직 중요!)
    # 시나리오: 로봇이 오른쪽으로 틀어짐 (Angle > 0)
    # -> 왼쪽 모터는 느려져야 하고 (또는 뒤로), 오른쪽 모터는 빨라져야(앞으로) 함
    
    # correction 값이 양수(+)일 때: "목표보다 각도가 작다(왼쪽을 보고 있다)" -> 오른쪽으로 돌려야 함
    # -> 왼쪽 모터 속도 증가, 오른쪽 모터 속도 감소
    
    # correction 값이 음수(-)일 때: "목표보다 각도가 크다(오른쪽을 보고 있다)" -> 왼쪽으로 돌려야 함
    # -> 왼쪽 모터 속도 감소, 오른쪽 모터 속도 증가
    
    # (참고: PID 수식에서 error = target - current 이므로, 
    #  오른쪽으로 틀어지면(current > 0), error는 음수, correction도 음수가 나옴)
    
    # 왼쪽 모터: 보정값이 음수(우회전상태)면 더 느리게 -> 더하기(+)를 하면 됨 (음수를 더하므로 감속)
    final_speed_l = BASE_SPEED + correction_l 
    
    # 오른쪽 모터: 보정값이 음수(우회전상태)면 더 빠르게 -> 빼기(-)를 하면 됨 (음수를 빼므로 가속)
    final_speed_r = BASE_SPEED - correction_r
    
    # 4. 모터 구동
    left_motor.run(final_speed_l)
    right_motor.run(final_speed_r)
    
    wait(10)