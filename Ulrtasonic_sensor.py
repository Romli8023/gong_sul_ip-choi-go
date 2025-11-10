#!/usr/bin/env python3
import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, SpeedPercent #OUTPUT_A 물리적인 포트 주소이기 떄문에 OUTPUT_B,C일수 도 있음
from ev3dev2.sensor.lego import UltrasonicSensor, INPUT_1 #INPUT_1 또한 물리적인 포트 주소이기에 INPUT_2,3,4일 수도 있음

# 1. 모터와 센서를 EV3의 어느 포트에 연결했는지 확인 후 수정***
# 예: 모터는 A 포트, 초음파 센서는 1번 포트

motor = LargeMotor(OUTPUT_A)
ultrasonic_sensor = UltrasonicSensor(INPUT_1)

# 2. 모터의 속도와 멈출 거리(임계값)를 설정

SPEED = 20  # 모터 속도 (0 ~ 100)
DISTANCE_THRESHOLD = 30  # 물체가 이 거리(mm)보다 가까워지면 모터가 멈춤


try:
    print("로봇팔을 움직입니다. 컵을 감지하면 멈춥니다.")
    
    # 1. 먼저 모터를 설정한 속도로 작동
    motor.on(SpeedPercent(SPEED))
    
    # 2. 무한 루프를 돌면서 거리를 계속 확인
    while True:
        # 현재 거리 측정
        current_distance = ultrasonic_sensor.distance_millimeters
        
        # 측정된 거리를 화면에 출력합니다. (확인용)
        print(f"현재 거리: {current_distance:.2f} mm")
        
        # 3. 만약 현재 거리가 설정한 임계값보다 작거나 같아지면,
        if current_distance <= DISTANCE_THRESHOLD:
            print("컵 감지! 모터를 정지합니다.")
            
            # 모터를 정지
            motor.off()
            
            # 루프를 빠져나와 프로그램을 종료합니다.
            break
        
        # 0.1초 동안 잠시 기다렸다가 다시 거리를 측정합니다. (CPU 과부하 방지)
        time.sleep(0.1)

except KeyboardInterrupt:
    # Ctrl+C를 눌러 프로그램을 강제 종료할 경우 모터를 안전하게 멈춥니다.
    print("프로그램이 강제 종료되었습니다.")
    motor.off()

#추가적으로 로봇팔 제어도 되어 있긴 하지만 일단 로봇팔 제어 코드 작성해 주세요.
