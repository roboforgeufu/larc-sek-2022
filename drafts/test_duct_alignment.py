#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.tools import StopWatch, wait

# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize.
motor_l = Motor(Port.B)
motor_r = Motor(Port.C)
# motorA = Motor(Port.A)
stopwatch = StopWatch()
color_l = ColorSensor(Port.S1)
color_r = ColorSensor(Port.S2)
ultra_l = UltrasonicSensor(Port.S3)
ultra_r = UltrasonicSensor(Port.S4)

WHEEL_DIAMETER = 5.5
WHEEL_DIST = 15.3
ROTATION = ((WHEEL_DIST) / WHEEL_DIAMETER) * 360
