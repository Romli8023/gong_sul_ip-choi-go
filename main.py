#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialize the EV3 Brick.
ev3 = EV3Brick()
motor_pitch = Motor(Port.A)
motor_yaw = Motor(Port.B)
gyro_pitch = GyroSensor(Port.S1)
gyro_yaw = GyroSensor(Port.S4)

def calibrate_gyro(gyro):
    gyro.reset_angle(0)
    wait(2000)
    gyro.reset_angle(0)
    ev3.speaker.beep()
calibrate_gyro(gyro_pitch)
calibrate_gyro(gyro_yaw)


Kp1 = 0.5
Kp2 = 0.05
Ki1 = 0.8
Ki2 = 0.5 
Kd1 = 0.05
Kd2 = 0.8

integral_1 = 0
integral_2 = 0
prev_error_1 = 0
prev_error_2 = 0
target_1 = 0
target_2 = 0

dt = 20

while True:
    angle_1 = gyro_pitch.angle()
    error_1 = target_1 - angle_1
    integral_1 += error_1*dt
    devitation_1 = (error_1 - prev_error_1) /dt
    prev_error_1 = error_1

    output_1 = Kp1*error_1 + Ki1*integral_1 + Kd1*devitation_1
    
    if output_1 >900: 
        output_1 =900
    elif output_1<-900: 
        output_1 =-900

    angle_2 = gyro_yaw.angle()
    error_2 = target_2 - angle_2
    integral_2 += error_2*dt
    devitation_2 = (error_2 - prev_error_2)/dt
    prev_error_2 = error_2

    output_2 = Kp2*error_2 +  Ki2*integral_2 + Kd2*devitation_2

    if output_2 >600: 
        output_2 =600
    elif output_2<-600: 
        output_2 =-600

    motor_pitch.run(output_1)
    motor_yaw.run (output_2)


    ev3.screen.clear()
    ev3.screen.print("Angle_1:",angle_1)
    ev3.screen.print("Output_1:",output_1)
    ev3.screen.print("Angle_2:",angle_2)
    ev3.screen.print("Output_2:",output_2)
    if abs(gyro_pitch.speed()) < 1 and abs(error_1) < 1:
        gyro_pitch.reset_angle(0)
    if abs(gyro_yaw.speed()) < 1 and abs(error_2) < 1:
        gyro_yaw.reset_angle(0)
    wait(dt)
