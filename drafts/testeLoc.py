#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.tools import StopWatch, wait
# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize.
motorB = Motor(Port.B)
motorC = Motor(Port.C)
motorA = Motor(Port.A)
cronometro = StopWatch()
sensorc1 = ColorSensor(Port.1)
sensorc2 = ColorSensor(Port.2)

def alinhar(vel):
    while(sensorc1.color()!=Color.BLACK and sensorc2.color()!=Color.BLACK):
        motorC.run(vel)
        motorB.run(vel) #identifica a linha preta

    motorC.hold()
    motorB.hold()

    while(sensorc2.color()!=Color.BLACK):
        motorC.run(vel)
    motorC.hold()
    while(sensorc2.color()!=Color.WHITE): #alinha o motor C
        motorC.run(vel)
    motorC.reset_angle(0)

    motorC.hold()
    motorC.run_target(vel,-50,then=Stop.HOLD)
    while(sensorc2.color()!=Color.WHITE):
        motorC.run(-vel)

    while(sensorc1.color()!=Color.BLACK):
        motorB.run(vel)
    motorB.hold()
    while(sensorc1.color()!=Color.WHITE): #alinha o motor B
        motorB.run(vel)
    motorB.reset_angle(0)

    motorB.hold()
    motorB.run_target(vel,-50,then=Stop.HOLD)
    while(sensorc1.color()!=Color.WHITE):
        motorB.run(-vel)

    motorB.hold()
    angB = motorB.angle()
    motorB.reset_angle(0)
    motorB.run_target(vel,-angB/2,then=Stop.HOLD)

    FC = 0.8 #fator de correcao
    motorC.hold()
    angC = motorC.angle()
    motorC.reset_angle(0)
    motorC.run_target(vel,(-angB/2)*FC,then=Stop.HOLD)

alinhar(100)