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
sensorinfra2 = InfraredSensor(Port.S2)

def anda_reto_graus(velBase,graus): #para dar ré os dois valores devem ser negativos
    Kp = 3                           
    Ki = 0.02
    Kd = 3 

    t = 0
    integ = 0
    erro = 0
    media = 0
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    cronometro.reset()
    if(graus<0):
        while(media>graus):
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
    else:
        while(media<graus):
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

# vel = 200
# flag = 0
# valoresI = []
# angM = 0
    
# motorB.run(vel)
# motorC.run(vel)
# wait(100)
# if(sensorinfra2.distance()<50):
#     print(sensorinfra2.distance())
#     motorB.reset_angle(0)
#     motorC.reset_angle(0)
#     while(sensorinfra2.distance()<50 and angM<300):
#         angM = abs(motorB.angle()+motorC.angle())/2
#         print(angM)
#         motorB.run(-vel)
#         motorC.run(-vel)
#     print("a")
#     while(sensorinfra2.distance()<70): # and sensorinfra3.distance()>10
#         print("b")
#         motorB.run(vel/3)
#         motorC.run(vel*2)
#     motorB.brake()
#     motorC.brake()
#     refI = sensorinfra3.distance()
#     while not flag:
#         motorB.run(-vel/2)
#         motorC.run(vel/2)
#         valoresI.append(sensorinfra3.distance())
#         if(len(valoresI)>5):
#             mediaI = sum(valoresI)/len(valoresI)
#             if(mediaI>refI): 
#                 motorB.brake()
#                 motorC.brake()
#                 flag = 1
#             valoresI.clear()

velB = 250
vel = velB
valorMin = 100
flag = 0

while True:
    motorB.run(velB*1.5)
    motorC.run(velB*0.5)
    wait(500)
    motorB.brake()
    motorC.brake()
    anda_reto_graus(velB,100)
    while not flag:
        motorC.run(velB)
        motorB.brake()
        if(sensorinfra3.distance()>valorMin):
            if(sensorinfra3.distance()>valorMin):
                flag = 1
        if(valorMin>sensorinfra3.distance()): valorMin = sensorinfra3.distance()
    flag = 0

    motorB.brake()
    motorC.brake()

