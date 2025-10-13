🚀 Lego EV3를 이용한 능동형 자동 수평 컵홀더 (Active Self-Balancing Cup Holder)
공학설계입문 과제 프로젝트로, Lego EV3와 자이로 센서를 활용하여 어떤 상황에서도 수평을 유지하는 컵홀더를 제작합니다.

🎯 프로젝트 목표 (Project Goal)
휠체어나 차량과 같이 흔들림이 잦은 이동수단에 컵홀더를 장착했을 때, 내부의 내용물이 밖으로 넘치지 않도록 스스로 균형을 조절하는 안전한 컵홀더를 제작하는 것을 목표로 합니다.


핵심 기능: <능동적 수평 유지 시스템> 

자이로 센서(IMU)로 컵홀더의 기울기를 실시간으로 감지합니다.

PID 제어 알고리즘을 통해 기울어진 각도를 보정하기 위한 모터 제어 값을 계산합니다.

4개의 모터가 즉각적으로 반응하여 경사로나 턱에서도 컵홀더의 수평을 완벽하게 유지합니다.

💡 기존 제품과의 차별점 (Benchmarking)
구분	
기존 클램프형 컵홀더 

본 프로젝트 (능동형 컵홀더)
가격	
저렴함 

상대적으로 고가
재질	
대부분 플라스틱 

Lego Technic 부품
각도 조절	
수동으로 고정 

센서와 모터로 자동 조절
안정성	
경사로나 충격에 매우 취약함 

실시간으로 기울어짐에 대응하여 안정적

Sheets로 내보내기
저희 프로젝트는 카메라 안정화 장비에 사용되는 짐벌 스테빌라이저 기술에 착안하여, IMU 센서와 모터를 이용해 이동 중에도 안정성을 유지하는 것을 핵심으로 합니다.

🛠️ 시스템 구성 (System Architecture)
하드웨어 (Hardware)
부품 (Component)	수량 (Quantity)	역할 (Role)
Lego EV3 Intelligent Brick	1	메인 컨트롤러 (PID 연산 및 제어)
Lego EV3 Gyro Sensor	1	X, Y축 기울기 각도 측정
Lego EV3 Large Motor	4	각 축의 균형을 맞추기 위한 구동
Lego Technic 부품	다수	2축 짐벌 구조 및 프레임 제작

Sheets로 내보내기
소프트웨어 (Software)
개발 언어: Python

라이브러리: ev3dev2

핵심 알고리즘: PID 제어

⚙️ 작동 원리 (How It Works)
프로젝트의 작동은 감지 → 계산 → 제어 의 세 단계로 이루어집니다.


실시간 감지 (Sense): 자이로 센서가 X축과 Y축의 현재 기울기 값을 지속적으로 측정합니다.

오차 계산 (Calculate): 측정된 값과 목표 값(수평 = 0°)의 차이, 즉 **오차(Error)**를 계산합니다.

모터 제어 (Control): 이 오차를 바탕으로 PID 제어 알고리즘이 모터를 얼마나 빠르고 강하게 움직여야 할지 계산한 후, 4개의 모터를 움직여 기울어진 방향의 반대로 자세를 바로잡습니다.

이 과정이 매우 짧은 시간 안에 반복되며 외부의 흔들림에 실시간으로 대응하여 균형을 유지합니다.

🧠 핵심 코드 (Core Logic)
전체 코드는 main.py 파일에서 확인하실 수 있으며, 핵심 로직인 PID 제어부와 메인 루프는 다음과 같습니다.
(Source: https://github.com/Romli8023/gong_sul_ip-choi-go/blob/main/main.py)

Python

# PID 제어 클래스
class PID:
    def __init__(self):
        # ... (초기 변수 설정) ...

    def set_pid(self, p, i, d):
        self.kp = p
        self.ki = i
        self.kd = d

    def pid_control(self, current_value):
        self.error = self.set_point - current_value
        self.p_control = self.kp * self.error
        self.i_control += self.ki * self.error * self.dt
        self.d_control = self.kd * (self.error - self.pre_error) / self.dt
        self.pre_error = self.error
        self.control = self.p_control + self.i_control + self.d_control
        return self.control

# 메인 제어 루프
try:
    while True:
        # 자이로 센서로부터 현재 각도 값 수신
        gyro_value = self_gyro.angle
        
        # PID 계산을 통해 모터 제어 속도 결정
        control = pid.pid_control(gyro_value)
        control_speed = control

        # 계산된 속도로 모터 구동 (두 쌍의 모터가 반대 방향으로 회전)
        motor_a.on(SpeedRPM(control_speed))
        motor_b.on(SpeedRPM(-control_speed))
        motor_c.on(SpeedRPM(control_speed))
        motor_d.on(SpeedRPM(-control_speed))

        time.sleep(pid.dt)

except KeyboardInterrupt:
    # 프로그램 종료 시 모터 정지
    motor_a.off()
    motor_b.off()
    motor_c.off()
    motor_d.off()

📅 진행 현황 및 계획 (Roadmap)
[x] 필요 기구 설계 및 조립 

[x] 센서 연동 및 기울기 데이터 확인 

[x] 제어 알고리즘 프로그래밍 (PID) 

[ ] 중간발표 

[ ] 추가 기능 구현 (휠체어 고정 장치 등) 

[ ] 전체 시스템 통합 및 안정화 

[ ] 최종 테스트 및 시연 영상 제작 
