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
# sensorultra4 = UltrasonicSensor(Port.S4)
sensorinfra3 = InfraredSensor(Port.S3)
sensorinfra2 = InfraredSensor(Port.S2)

def pega_parede():

    vel = 200
    flag = 0
    valoresI = []
    angM = 0
    mediaMin = 100
    
    while(sensorinfra2.distance()>40 and sensorinfra2.distance()>55):
        motorB.run(vel)
        motorC.run(vel)
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    while(sensorinfra2.distance()<40 and angM<300):
        angM = abs(motorB.angle()+motorC.angle())/2
        print(angM)
        motorB.run(-vel)
        motorC.run(-vel)
    while(sensorinfra2.distance()<60): # and sensorinfra3.distance()>10
        motorB.run(vel/3)
        motorC.run(vel*2)
    motorB.brake()
    motorC.brake()
    while not flag:
        motorB.run(-vel/2)
        motorC.run(vel/2)
        valoresI.append(sensorinfra3.distance())
        if(len(valoresI)>5):
            mediaI = sum(valoresI)/len(valoresI)
            if(mediaI>mediaMin): 
                motorB.brake()
                motorC.brake()
                flag = 1
            if(mediaMin>mediaI): mediaMin = mediaI
            valoresI.clear()

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

def anda_buraco(velBase,modo): #1 até ver #2 até deixar de ver a parede
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
    if(modo==1):
        while(sensorinfra3.distance()>25):
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
    if(modo==2):
        while(sensorinfra3.distance()<25):
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

valoresM = []
valoresI = []
valorMin = 100
vel = 300
dist = 15
K1 = 5
K2 = 0.5


#pega_parede()
while True:
    motorB.run(vel)
    velB = motorB.speed()

    difMotor = motorB.angle() - motorC.angle()
    valoresM.append(difMotor)
    
    difInfra = sensorinfra3.distance() - dist
                
    if(sensorinfra3.distance()>100):
        # if(sensorultra4.distance()<25):
        #     print("curva")
        # else:
        print("buraco")
        break 

    elif(difInfra<0):
        motorC.run(velB*1.25+K1*abs(difInfra))

    elif(difInfra>=0):
        motorB.run(velB*1.25)
        motorC.run(velB*0.75)
        wait(1000)
        motorB.brake()
        motorC.brake()
        anda_reto_graus(velB,200)
        
        while not flag:
            motorB.run(-vel/2)
            motorC.run(vel/2)
            valoresI.append(sensorinfra3.distance())
            if(len(valoresI)>5):
                mediaI = sum(valoresI)/len(valoresI)
                if(mediaI>mediaMin): 
                    motorB.brake()
                    motorC.brake()
                    flag = 1
                if(mediaMin>mediaI): mediaMin = mediaI
                valoresI.clear()
        
        motorB.brake()
        motorC.brake()
        


motorB.brake()
motorC.brake()
anda_buraco(-100,1)
anda_buraco(100,2)
print(mede_buraco_cm(100),"cm")

# if(len(valoresM)>10): 
#     media = sum(valoresM)/len(valoresM)
#     if(media>20): 
#         motorC.run(velB*1.1)
#         wait(50)
#     valoresM.clear()

# motorC.run(velB*0.75-K2*abs(difInfra))
#         valoresI.append(sensorinfra3.distance())
#         if(difMotor>20 and sensorinfra3.distance()<25):
#             if(sensorinfra3.distance()>valorMin):
#                 cronometro.reset()
#                 while(sensorinfra3.distance()>valorMin and cronometro.time()<50):
#                     motorC.run(velB)
#                     motorB.brake()
#             if(valorMin>sensorinfra3.distance()): valorMin = sensorinfra3.distance()