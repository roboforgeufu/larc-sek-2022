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
# motorA = Motor(Port.A)
cronometro = StopWatch()
sensorc1 = ColorSensor(Port.S1)
sensorc2 = ColorSensor(Port.S2)

def dc_acel(time,modo): #modo 1 termina vel 0 modo 2 vel max modo 3 começa vel max termina vel 0

    Kp = 3 
    Ki = 0.02
    Kd = 3

    t = 0
    integ = 0
    erro = 0
    reflex_saida = 75

    cronometro.reset()
    motorB.reset_angle(0)
    motorC.reset_angle(0)

    if(modo==1):
        while(cronometro.time()<time):

            erro0 = erro
            erro = motorC.angle() - motorB.angle()
            prop = erro*Kp 

            if(-3<erro<3): integ = integ+(erro*Ki)

            t0 = t
            wait(1)
            t = cronometro.time()
            tempoDecor = t - t0
            deriv = ((erro - erro0)*Kd)/tempoDecor

            correcao = prop+integ+deriv
            vel = -(cronometro.time()*20/time)**2+cronometro.time()*(400/time)+20

            motorB.dc(vel+correcao)
            motorC.dc(vel-correcao)

        motorB.hold()
        motorC.hold()

    if(modo==2):
        while(cronometro.time()<time):

            erro0 = erro
            erro = motorC.angle() - motorB.angle()
            prop = erro*Kp 

            if(-3<erro<3): integ = integ+(erro*Ki)

            t0 = t
            wait(1)
            t = cronometro.time()
            tempoDecor = t - t0
            deriv = ((erro - erro0)*Kd)/tempoDecor

            correcao = prop+integ+deriv
            vel = -(cronometro.time()*10/time)**2+cronometro.time()*(200/time)+20

            motorB.dc(vel+correcao)
            motorC.dc(vel-correcao)
            if(sensorc1.reflection()<reflex_saida or sensorc2.reflection()<reflex_saida): break

        motorB.hold()
        motorC.hold()

    if(modo==3):
        while(cronometro.time()<time):

            erro0 = erro
            erro = motorC.angle() - motorB.angle()
            prop = erro*Kp 

            if(-3<erro<3): integ = integ+(erro*Ki)

            t0 = t
            wait(1)
            t = cronometro.time()
            tempoDecor = t - t0
            deriv = ((erro - erro0)*Kd)/tempoDecor

            correcao = prop+integ+deriv
            vel = -(cronometro.time()*10/time)**2+100

            motorB.dc(vel+correcao)
            motorC.dc(vel-correcao)
            if(sensorc1.reflection()<reflex_saida or sensorc2.reflection()<reflex_saida): break

        motorB.hold()
        motorC.hold()


def buraco():

    vel=100
    flag=0
    flag2=0
    tempo=100
    
    reflex_saida = 75 #maior do que o maior valor de reflexao entre as cores e a rampa e menor que o branco
    reflex_branco = 85 #maior que o valor acima pra identificar o branco
    reflex_buraco = 10 #maior que o valor do buraco (0) e menor do que o menor valor entre as cores e a rampa

    if(sensorc1.reflection()<=reflex_saida):
        cronometro.reset()
        if(sensorc1.reflection()>10):
            while(cronometro.time()<tempo):
                motorB.dc(vel/3)
                motorC.dc(vel/3)
            motorC.hold()
            motorB.hold()
            wait(100)
            flag2=1
        if(sensorc1.reflection()<reflex_buraco): 
            ev3.speaker.beep()
            flag=1
        wait(100)
        if(sensorc1.color()!=None and sensorc1.color()!=Color.WHITE):
            print(sensorc1.color())
            ev3.speaker.beep(1000)
            flag=2
        wait(100)
        cronometro.reset()
        if not flag2:
            while(cronometro.time()<tempo):
                motorB.dc(-vel/3)
                motorC.dc(-vel/3)
            motorC.hold()
            motorB.hold()
        while(sensorc2.reflection()>reflex_saida):
            motorC.dc(vel/2)
        motorC.hold()
        while(sensorc1.reflection()<reflex_branco):
            motorB.dc(-vel/3)
        motorB.hold()
        while(sensorc2.reflection()<reflex_branco):
            motorC.dc(-vel/3)   

    else:
        cronometro.reset()
        if(sensorc1.reflection()>10):
            while(cronometro.time()<tempo):
                motorB.dc(vel/3)
                motorC.dc(vel/3)
            motorC.hold()
            motorB.hold()
            wait(100)
            flag2=1
        if(sensorc2.reflection()<reflex_buraco): 
            ev3.speaker.beep()
            flag=1
        wait(100)
        if(sensorc2.color()!=None and sensorc2.color()!=Color.WHITE):
            ev3.speaker.beep(1000)
            flag=2
        cronometro.reset()
        if not flag2:
            while(cronometro.time()<tempo):
                motorB.dc(-vel/3)
                motorC.dc(-vel/3)
            motorC.hold()
            motorB.hold()
        while(sensorc1.reflection()>reflex_saida):
            motorB.dc(vel/2)
        motorB.hold()
        while(sensorc2.reflection()<reflex_branco):
            motorC.dc(-vel/3)
        motorC.hold()
        while(sensorc1.reflection()<reflex_branco):
            motorB.dc(-vel/3)

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
vel=100
reflex_saida = 75

dc_acel(1000,2)
cronometro.reset()
time=800
while(cronometro.time()<time):
    velB = (cronometro.time()*10/time)**2-cronometro.time()*(200/time)+20
    print(velB)
    if(velB<20): velB=20
    motorB.dc(velB)
    motorC.dc(vel)
    if(sensorc1.reflection()<reflex_saida or sensorc2.reflection()<reflex_saida): break
motorC.brake()
motorB.brake()
dc_acel(10000,3)
motorB.dc(0)
motorB.dc(0)
print(buraco())
