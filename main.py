#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop

# 1. EV3 브릭 및 모터 초기화
ev3 = EV3Brick()

gripper_motor = Motor(Port.A) #포트에 따라서 A,B, C일수 있음

# 2. 모터 작동 및 정지 알고리즘
# 모터 회전 속도 (deg/s)
SPEED = 45

# 작동 시간 (ms 단위, 1000ms = 1초)
# 1500ms는 1.5초를 의미합니다.
RUN_TIME_MS = 1000

# 지정된 속도로 지정된 시간만큼 모터를 작동시킨 후 자동으로 멈춥니다.
# 마지막 인수인 'then=Stop.HOLD'는 모터가 멈춘 후 그 위치를 유지하도록 하여,
# 컵을 꽉 잡은 상태를 유지하는 데 도움이 됩니다.
gripper_motor.run_time(SPEED, RUN_TIME_MS, then=Stop.HOLD, wait=True)

# 작업이 완료되었음을 알리는 비프음
ev3.speaker.beep()
