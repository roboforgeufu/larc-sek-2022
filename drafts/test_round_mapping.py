#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (
    ColorSensor,
    GyroSensor,
    InfraredSensor,
    Motor,
    TouchSensor,
    UltrasonicSensor,
)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port
from pybricks.tools import DataLog, StopWatch, wait

ev3 = EV3Brick()
motorB = Motor(Port.B)
motorC = Motor(Port.C)
sensorinfra3 = InfraredSensor(Port.S3)
gyro = GyroSensor(Port.S1)
cronometro = StopWatch()


vel = 100

motorB.reset_angle(0)
motorC.reset_angle(0)

logger = DataLog("motorB", "motorC", "infra", "gyro", "time")

while abs(motorB.angle()) < 2050 and abs(motorC.angle()) < 2050:
    motorB.dc(vel)
    motorC.dc(-vel)
    logger.log(
        motorB.angle(),
        motorC.angle(),
        sensorinfra3.distance(),
        gyro.angle(),
        cronometro.time(),
    )

motorB.hold()
motorC.hold()
