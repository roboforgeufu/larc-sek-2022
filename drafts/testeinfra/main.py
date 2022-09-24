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
sensorultra2 = UltrasonicSensor(Port.S2)
sensorinfra3 = InfraredSensor(Port.S3)
sensorinfra4 = InfraredSensor(Port.S4)

dist = 10
vel = 100
valores = []
while True:
    difMotor = motorB.speed() - motorC.speed()
    difInfra = sensorinfra3.distance() - dist
    motorC.run(vel)
    if(abs(difMotor)<20):
        print("M:", difMotor, "\tI:", difInfra, "\t|", "OK")
        motorB.run(vel)
        if(abs(difInfra)>2):
            motorB.run(vel+40*(difInfra/difInfra))
    elif(difMotor>0):
        print("M:", difMotor, "\tI:", difInfra, "\t|", "Aproximando..")
        motorB.run(vel+40)
        if(sensorinfra3.distance()<dist*0.75):
            motorB.run(vel-40)
    else:
        print("M:", difMotor, "\tI:", difInfra, "\t|", "Afastando..")
        motorB.run(vel-40)
        if(sensorinfra3.distance()>dist*1.25):
            motorB.run(vel+40)
