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
sensorc1 = ColorSensor(Port.S1)
sensorc2 = ColorSensor(Port.S2)

def buraco(vel):
    flag=0
    tempo=(3200-5*vel)/2
    while(sensorc1.reflection()>60 and sensorc2.reflection()>60):
        motorC.run(vel)
        motorB.run(vel)
    motorC.brake()
    motorB.brake()

    if(sensorc1.reflection()<=60):
        cronometro.reset()
        while(cronometro.time()<tempo):
            velT = vel - cronometro.time()
            if(velT<vel/3): velT = vel/3
            motorB.run(velT)
            motorC.run(velT)
        motorC.hold()
        motorB.hold()
        wait(200)
        if(sensorc1.reflection()<10): 
            ev3.speaker.beep()
            flag=1
        wait(200)
        if(sensorc1.color()!=None and sensorc1.color()!=Color.WHITE):
            print(sensorc1.color())
            ev3.speaker.beep(1000)
            flag=2
        cronometro.reset()
        while(cronometro.time()<tempo):
            motorB.run(-velT)
            motorC.run(-velT)
        motorC.hold()
        motorB.hold()
        while(sensorc2.reflection()>60):
            motorC.run(vel)
        motorC.hold()
        while(sensorc1.reflection()<85):
            motorB.run(-vel/2)
        motorB.hold()
        while(sensorc2.reflection()<85):
            motorC.run(-vel/2)   

    else:
        cronometro.reset()
        while(cronometro.time()<tempo):
            velT = vel - cronometro.time()
            if(velT<vel/3): velT = vel/3
            motorB.run(velT)
            motorC.run(velT)
        motorC.hold()
        motorB.hold()
        wait(200)
        if(sensorc2.reflection()<10): 
            ev3.speaker.beep()
            flag=1
        wait(200)
        if(sensorc2.color()!=None and sensorc2.color()!=Color.WHITE):
            ev3.speaker.beep(1000)
            flag=2
        cronometro.reset()
        while(cronometro.time()<tempo):
            motorB.run(-velT)
            motorC.run(-velT)
        motorC.hold()
        motorB.hold()
        while(sensorc1.reflection()>60):
            motorB.run(vel)
        motorB.hold()
        while(sensorc2.reflection()<85):
            motorC.run(-vel/2)
        motorC.hold()
        while(sensorc1.reflection()<85):
            motorB.run(-vel/2)

    motorB.brake()
    motorC.brake()
    return flag

def curva(angulo): #angulo positivo: direita, negativo: esquerda
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    Kp = 3
    Ki = 0.2
    Kd = 8
    
    t = 0
    integ = 0
    erro = 0
    while(motorB.angle()!=angulo):
        media = (motorB.angle() - motorC.angle())/2
        erro0 = erro
        erro = angulo - media

        prop = erro*Kp
        if(-3<erro<3): integ = integ+(erro*Ki)
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((erro - erro0)*Kd)/tempoDecor

        correcao = prop+integ+deriv
        vel = 5 + correcao
        if(vel<0):
            if(vel>-5): vel = -5
        else:
            if(vel<5): vel = 5
        motorC.run(-vel)
        motorB.run(vel)

    motorC.brake()
    motorB.brake()

def anda_reto_graus(velBase,graus,stop): #para dar ré os dois valores devem ser negativos
    Kp = 3                               #stop 1: hold 2: brake 3: coast
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
    if(stop==1):
        motorC.hold()
        motorB.hold()
    if(stop==2):
        motorC.brake()
        motorB.brake()
    if(stop==3):
        return

distRodas = 15.2
diamRodas = 5.7
rotacao = ((distRodas)/diamRodas)*360
flag=0
while flag!=1:
    flag = buraco(300)
    wait(100)
    anda_reto_graus(-300,-100,1)
    wait(200)
    curva(round(rotacao/4))
    wait(100)


    
