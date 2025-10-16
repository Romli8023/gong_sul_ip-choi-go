from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Color
import time

# --- 1. 상수 및 하드웨어 객체 생성 ---

# 비례 제어 상수 (1.0 ~ 1.5 사이에서 조절하며 최적의 값을 찾으세요)
P_GAIN = 1.2

# EV3 브릭 및 하드웨어 초기화 (연결된 포트에 맞게 수정하세요)
ev3 = EV3Brick()
motor_pitch = Motor(Port.A)
motor_roll = Motor(Port.B)
gyro_pitch = GyroSensor(Port.S1)
gyro_roll = GyroSensor(Port.S2)


# --- 2. 초기화 함수 ---

def initialize():
    """프로그램 시작 시 모터와 센서를 초기화합니다."""
    print("Initializing 2-axis gimbal...")
    ev3.light.on(Color.ORANGE)
    ev3.speaker.beep()

    # 두 모터를 중앙(0도)으로 이동 (완료될 때까지 대기)
    motor_pitch.run_target(speed=800, target_angle=0, wait=True)
    motor_roll.run_target(speed=800, target_angle=0, wait=True)
    
    # 자이로 센서가 안정될 시간을 주고, 현재 각도를 0으로 리셋
    print("Resetting gyro sensors...")
    time.sleep(0.5)
    gyro_pitch.reset_angle(0)
    gyro_roll.reset_angle(0)
    time.sleep(0.5)

    ev3.light.on(Color.GREEN)
    ev3.speaker.beep(frequency=800, duration=200)
    print("Initialization complete. System is active.")


# --- 3. 메인 프로그램 실행 ---

# 시작 시 초기화 함수를 1회 실행
initialize()

# 무한 루프 시작
while True:
    # 3-1. 각 축의 현재 기울기 각도 읽기
    pitch_angle = gyro_pitch.angle()
    roll_angle = gyro_roll.angle()
    
    # 3-2. 각 모터의 목표 각도 계산 (기울어진 반대 방향으로)
    target_pitch = int(pitch_angle * -P_GAIN)
    target_roll = int(roll_angle * -P_GAIN)

    # 3-3. 계산된 목표 각도로 각 모터를 이동 (wait=False로 루프가 멈추지 않게 함)
    motor_pitch.run_target(speed=1000, target_angle=target_pitch, wait=False)
    motor_roll.run_target(speed=1000, target_angle=target_roll, wait=False)

    # 3-4. CPU에 과부하를 주지 않도록 짧은 지연시간 추가
    time.sleep(0.01)