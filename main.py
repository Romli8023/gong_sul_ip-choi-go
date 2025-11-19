#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port,Stop


# ====== CONSTANTS ======
MAX_INTEGRAL = 100
MAX_OUTPUT = 720
DEADBAND = 1

LOOP_DT_MS = 10
LOOP_DT_S = LOOP_DT_MS / 1000

# ====== PID GAINS TRỤC ROLL (OX) ======
Kp_R = 2.0
Ki_R = 0.04
Kd_R = 0.6

# ====== PID GAINS TRỤC PITCH (OY) ======
Kp_P = 3.0
Ki_P = 0.015
Kd_P = 0.55

ev3 = EV3Brick()

# ====== HARDWARE ======
motor_roll  = Motor(Port.A)   # Trục X
motor_pitch = Motor(Port.B)   # Trục Y

gyro_roll  = GyroSensor(Port.S3)
gyro_pitch = GyroSensor(Port.S4)

integral_roll = 0
prev_error_roll = 0
target_roll = 0

integral_pitch = 0
prev_error_pitch = 0
target_pitch = 0



# ====== GYRO INIT + BIAS ======
def init_gyro_and_bias(gyro, name):
    ev3.screen.clear()
    ev3.screen.print("Init", name)
    ev3.speaker.beep()

    # ĐẶT HOÀN TOÀN YÊN, KHÔNG ĐỤNG!
    wait(3000)
    gyro.reset_angle(0)
    wait(3000)  # cho nó ổn định thêm

    ev3.screen.clear()
    ev3.screen.print("Measuring", name)

    bias = 0
    samples = 300
    total = 0
    for _ in range(samples):
        total += gyro.speed()
        wait(10)
    bias = total / samples

    ev3.screen.clear()
    ev3.screen.print(name, "Bias:", int(bias))
    ev3.speaker.beep(frequency=800, duration=200)

    return bias


bias_roll  = init_gyro_and_bias(gyro_roll,  "Roll")
bias_pitch = init_gyro_and_bias(gyro_pitch, "Pitch")


# ====== STATE VARIABLES ======
angle_roll = 0

angle_pitch = 0


# ---------------------------
# ------- MAIN LOOP ---------
# ---------------------------
while True:

    # ====== READ GYRO RATE ======
    rate_roll  = gyro_roll.speed()  - bias_roll
    rate_pitch = gyro_pitch.speed() - bias_pitch

    # ====== INTEGRATE ANGLE ======
    angle_roll  += rate_roll  * LOOP_DT_S
    angle_pitch += rate_pitch * LOOP_DT_S

    # ====== ERROR ======
    error_roll  = angle_roll  - target_roll
    error_pitch = angle_pitch - target_pitch

    if abs(error_roll) < DEADBAND: error_roll = 0
    if abs(error_pitch) < DEADBAND: error_pitch = 0

    # ====== PID ROLL (Ox) ======
    P_r = Kp_R * error_roll

    integral_roll += error_roll
    integral_roll = max(min(integral_roll, MAX_INTEGRAL), -MAX_INTEGRAL)
    I_r = Ki_R * integral_roll

    D_r = Kd_R * (error_roll - prev_error_roll)
    prev_error_roll = error_roll

    output_roll = max(min(P_r + I_r + D_r, MAX_OUTPUT), -MAX_OUTPUT)

    # ====== PID PITCH (Oy) ======
    P_p = Kp_P * error_pitch

    integral_pitch += error_pitch
    integral_pitch = max(min(integral_pitch, MAX_INTEGRAL), -MAX_INTEGRAL)
    I_p = Ki_P * integral_pitch

    D_p = Kd_P * (error_pitch - prev_error_pitch)
    prev_error_pitch = error_pitch

    output_pitch = max(min(P_p + I_p + D_p, MAX_OUTPUT), -MAX_OUTPUT)

    # ====== MOTOR OUTPUT ======
    motor_roll.run(-output_roll)
    motor_pitch.run(-output_pitch)

    # ====== DEBUG ======
    ev3.screen.clear()
    ev3.screen.print("Rate roll:", int(rate_roll))
    ev3.screen.print("Ang* roll:", int(angle_roll))
    ev3.screen.print("Err:", int(error_roll))
    ev3.screen.print("Out:", int(output_roll))
    ev3.screen.print("Bias:", int(bias_roll))

    wait(LOOP_DT_MS)