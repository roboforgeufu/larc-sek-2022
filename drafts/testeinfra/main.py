#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()

# Write your program here.

motorB = Motor(Port.B)
motorC = Motor(Port.C)
motorA = Motor(Port.A)
cronometro = StopWatch()
sensorinfra3 = InfraredSensor(Port.S3)
sensorinfra4 = InfraredSensor(Port.S4)

while(sensorinfra3.distance()<30):
    correcao = sensorinfra3.distance() - 5
    if(abs(correcao)>5):
        motorB.run(50 + 10*correcao)
        motorC.run(50 - 10*correcao)
    else:
        motorB.run(100)
        motorC.run(100)
motorB.hold()
motorC.hold()