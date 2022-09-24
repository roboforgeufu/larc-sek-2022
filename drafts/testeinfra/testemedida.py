#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

motorB = Motor(Port.B)
motorC = Motor(Port.C)
motorA = Motor(Port.A)
sensorinfra3 = InfraredSensor(Port.S3)
cronometro = StopWatch()

def mede_buraco_cm(velBase): 
    Kp = 3                               
    Ki = 0.1
    Kd = 5

    t = 0
    integ = 0
    erro = 0
    media = 0
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    cronometro.reset()
    while(sensorinfra3.distance()>20):
        media = (motorB.angle() + motorC.angle())/2
        erro0 = erro
        erro = motorC.angle() - motorB.angle()

        prop = erro*Kp 
        if(-3<erro<3): integ = integ+(erro*Ki)
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd)/tempoDecor

        correcao = prop+integ+deriv
        motorC.run(velBase-correcao)
        motorB.run(velBase+correcao)
    motorC.brake()
    motorB.brake()

    diamRodas = 5.5
    compRodas = diamRodas * 3.14159265359 #360 graus = 1 rotacao; 1 rotacao = 17.3cm
    graus = (motorB.angle()+motorC.angle())/2
    return (graus/360)*compRodas

print(mede_graus(300))