from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch

# 설정 (하드웨어 포트 확인)
# 모터 포트
LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.B

# 자이로 센서 포트 (2개 사용)
LEFT_GYRO_PORT = Port.S3
RIGHT_GYRO_PORT = Port.S4

# 기본 속도 및 목표 각도
BASE_SPEED = 200     # 기본 주행 속도
TARGET_ANGLE = 0     # 목표 각도 (0도 유지 = 직진)

# PID 게인 값 설정
KP = 1.2   # 비례 (P): 오차에 비례하여 반응 (반응 속도)
KI = 0.001 # 적분 (I): 누적 오차 보정 (미세 오차 제거)
KD = 0.5   # 미분 (D): 오차 변화율 예측 (급격한 변화 억제, 진동 방지)

# PID 클래스 
class PIDController:
    def __init__(self, kp, ki, kd, target):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        
        self.prev_error = 0
        self.integral = 0
        self.watch = StopWatch() # 시간 측정을 위한 스톱워치
        self.watch.reset()

    def compute(self, current_value):
        # dt (시간 변화량) 계산 - 초 단위
        dt = self.watch.time() / 1000.0 
        self.watch.reset()
        
        # dt가 너무 작으면(0이면) 계산 오류 방지
        if dt <= 0: dt = 0.001

        # 1. 오차(Error) 계산
        error = self.target - current_value
        
        # 2. 비례(P) 항
        p_term = error * self.kp
        
        # 3. 적분(I) 항
        self.integral += error * dt
        i_term = self.integral * self.ki
        
        # 4. 미분(D) 항
        derivative = (error - self.prev_error) / dt
        d_term = derivative * self.kd
        
        # 5. 업데이트
        self.prev_error = error
        
        # 6. 최종 출력 (Turn Value)
        output = p_term + i_term + d_term
        return output

# 초기화
ev3 = EV3Brick()
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

# 시작 알림
ev3.speaker.beep()
print("PID Control Started...")

# 4. 메인 루프 (듀얼 센서 피드백)
while True:
    # 1. 센서 값 읽기 (반사광)
    current_light = line_sensor.reflection()
    
    # 2. PID 계산 (조향 값 산출)
    # 목표값보다 어두우면 -> 한쪽으로 회전
    # 목표값보다 밝으면 -> 반대로 회전
    turn_rate = steering_pid.compute(current_light)
    
    # 3. 모터 파워 계산 (2축 제어: 속도 + 조향)
    
    # 출력 제한 (Clamping): 너무 급격한 회전 방지 (옵션)
    # if turn_rate > 100: turn_rate = 100
    # if turn_rate < -100: turn_rate = -100
    
    left_speed = BASE_SPEED + turn_rate
    right_speed = BASE_SPEED - turn_rate
    
    # 4. 모터 구동
    left_motor.run(left_speed)
    right_motor.run(right_speed)
    
    # 5. 연산 주기 조절 (너무 빠르면 센서 노이즈에 민감해짐)
    wait(10)