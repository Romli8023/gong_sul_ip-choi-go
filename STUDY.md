자이로 센서와 PID 제어 학습 정리1. 자이로 센서의 측정 원리자이로 센서(Gyro Sensor)는 기본적으로 **각속도(Angular Velocity)**를 측정한다. 즉, 물체가 '얼마나 빠른 속도로 회전하고 있는가'를 감지하는 센서다.하지만 수평 유지 장치(Balancing Robot)를 제어하기 위해 필요한 값은 회전 속도가 아니라 **현재의 기울어진 각도(Total Angle)**이다. 이를 구하기 위해 다음과 같은 적분 과정을 거친다.센서가 아주 짧은 시간($dt$) 동안의 회전 속도를 측정한다.이 속도 값들을 계속 누적(더하기)하여 **총 이동 거리(각도)**를 계산한다.MicroPython의 gyro_roll.angle() 함수가 이 내부적인 적분 과정을 처리하여, 초기 상태 대비 현재 얼마나 회전했는지에 대한 각도 값을 반환한다.2. PID 제어 알고리즘 (PID Control)PID 알고리즘은 로봇의 목표 상태인 **수평($\text{target} = 0$)**을 달성하기 위해, 현재 상태($\text{angle}$)를 바탕으로 모터의 출력($\text{output}$)을 조절하는 피드백 제어 방식이다.while True 루프를 통해 끊임없이 오차를 수정하며 다음 3가지 항(P, I, D)을 조합하여 제어한다.2.1. P 제어 (Proportional, 비례)현재 발생한 오차에 비례하여 반대 방향으로 힘을 가한다. '스프링'과 유사한 역할을 한다.$$ \text{error} = \text{target} - \text{angle} $$$$ \text{output}_p = K_p \times \text{error} $$작동 원리: 기울어진 만큼 반대로 밀어낸다.예: 각도가 $+10^\circ$ 기울어지면 ($\text{error} = -10$), 모터는 음(-)의 방향으로 힘을 낸다.예: 각도가 $+30^\circ$ 기울어지면, $K_p$에 비례하여 10도일 때보다 3배 더 강한 힘으로 밀어낸다.한계: $K_p$만으로는 미세한 오차나 로봇의 무게 중심 불균형으로 인해 목표값($0$)에 완전히 도달하지 못하는 잔류 오차(Steady-state error)가 남을 수 있다.2.2. I 제어 (Integral, 적분)오차가 해결되지 않고 지속되면, 시간이 지날수록 힘을 누적시켜 더 강하게 밀어준다.$$ \text{integral} \leftarrow \text{integral} + \text{error} $$$$ \text{output}_i = K_i \times \text{integral} $$작동 원리: $K_p$만으로 해결되지 않는 미세한 오차를 없애기 위해 사용된다. 오차가 계속 유지되면 $\text{integral}$ 값이 커지면서 모터 출력이 점점 강해져 결국 목표지점인 0으로 밀어 올린다.주의점: 적분값이 과도하게 쌓이면 목표 지점에 도달해서도 멈추지 못하고 반대편으로 튀어 나가는 진동(Overshoot)을 유발할 수 있다.2.3. D 제어 (Derivative, 미분)오차가 변화하는 속도를 감지하여 급격한 움직임에 제동을 건다. '댐퍼(Damper)' 또는 '브레이크' 역할을 한다.$$ \text{deviation} = \text{error} - \text{prev_error} $$$$ \text{prev_error} \leftarrow \text{error} $$$$ \text{output}_d = K_d \times \text{deviation} $$작동 원리: '오차가 얼마나 빠르게 변하고 있는가?'를 계산한다.로봇이 목표 지점(0도)을 향해 너무 빠른 속도로 돌진하면 $\text{deviation}$ 값이 커진다.$K_d$ 항은 이 돌진하는 힘에 반대되는 제동력을 발생시켜, 목표 지점을 지나치기 전에 미리 속도를 줄여 안정적으로 멈추게 한다.3. 최종 구현 코드 로직위의 3가지 제어 방식을 합산하여 최종 모터 출력을 결정한다.Python# 1. P, I, D의 힘을 모두 합산하여 최종 명령 생성
output = (Kp * error) + (Ki * integral) + (Kd * deviation)

# 2. 최종 명령(output)을 모터의 '속도' 또는 '파워'로 전달
motor_roll.run(output)

# 3. 주기 조절 (0.01초)
wait(10)
이 과정이 wait(10)에 의해 0.01초마다 반복되면서, 외부 충격이나 기울어짐에 실시간으로 반응하여 수평을 유지한다.4. 트러블슈팅 및 해결 (Troubleshooting)실제 튜닝 과정($K_p, K_i, K_d$ 조정)에서 발생한 문제와 해결 방법이다.4.1. 적분 와인드업 (Integral Windup)문제: $K_i$ 값을 적용하여 테스트하던 중, 시간이 지날수록 각도가 계속 증가하거나 모터가 통제 불능 상태로 회전하는 현상이 발생함. 이는 I항(적분값)이 제한 없이 너무 커져서 시스템이 불안정해졌기 때문임.해결: 적분값(integral)에 상한선(Clamping)을 두어 과도한 누적을 방지하는 안티 와인드업(Anti-Windup) 코드를 적용함.PythonMAX_INTEGRAL = 50

if integral > MAX_INTEGRAL:
    integral = MAX_INTEGRAL
elif integral < -MAX_INTEGRAL:
    integral = -MAX_INTEGRAL
4.2. 자이로 드리프트 (Gyro Drift)문제: 로봇을 가만히 세워두어도 소프트웨어가 계산한 각도($\text{angle}$) 값이 0에 머무르지 않고 천천히 증가하거나 감소하는 현상. 센서 내부의 오차 누적으로 인해 발생한다.해결:프로그램 시작 시 센서를 절대 움직이지 않는 상태에서 초기화한다.필요시 gyro_roll.reset_angle(0) 함수를 사용하여 현재 각도를 0점으로 재설정한다.
