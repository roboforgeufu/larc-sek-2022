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

def forward_while_same_reflection(speed_r=50, speed_l=50, reflection_diff=10):
    starting_ref_r = sensorc2.reflection()
    starting_ref_l = sensorc1.reflection()

    stopped_l = False
    stopped_r = False
    while not stopped_l or not stopped_r:
        diff_ref_r = sensorc2.reflection() - starting_ref_r
        diff_ref_l = sensorc1.reflection() - starting_ref_l
        if abs(diff_ref_r) < reflection_diff:
            motorC.dc(speed_r)
        else:
            if not stopped_l:
                stopped_l = True
            motorC.hold()

        if abs(diff_ref_l) < reflection_diff:
            motorB.dc(speed_l)
        else:
            if not stopped_r:
                stopped_r = True
            motorB.hold()
    motorB.dc(0)
    motorC.dc(0)

def accurate_color(rgb_tuple):
    if sum(rgb_tuple) == 0:
        return None
    if rgb_tuple[0] > 40 and rgb_tuple[1] > 40 and rgb_tuple[2] > 40:
        return Color.WHITE
    if rgb_tuple[0] in range(60, 75) and rgb_tuple[1] in range(30, 45):
        return Color.YELLOW
    if rgb_tuple[0] < 15 and rgb_tuple[1] > 15 and rgb_tuple[2] < 15:
        return Color.GREEN
    if rgb_tuple[0] < 15 and rgb_tuple[1] < 15 and rgb_tuple[2] < 15:
        return Color.BLACK


def check_land_position_by_color(sensorc1: ColorSensor, sensorc2: ColorSensor) -> str:
    color_left = accurate_color(sensorc1.rgb())
    color_right = accurate_color(sensorc2.rgb())

    if color_left == Color.GREEN:
        pos_left = "RAMP"
    elif color_left is None:
        pos_left = "EDGE"
    else:
        pos_left = str(color_left)

    if color_right == Color.GREEN:
        pos_right = "RAMP"
    elif color_right is None:
        pos_right = "EDGE"
    else:
        pos_right = str(color_right)

    if pos_left.startswith("Color") and pos_right.startswith("Color"):
        return "COLOR"
    if pos_left == pos_right:
        return pos_left

    return str(pos_left + ":" + pos_right)


def segue_linha_c1(vel,tempo):
    valorLum = 35 #medir na linha toda vez
    Kp = 3.5
    Ki = 0.05 
    Kd = 10

    erro = 0
    valorI = 0
    t = 0
    cronometro.reset()
    while True:
        erro0 = erro  
        erro = valorLum - sensorc1.reflection()  
        valorP = erro*Kp
        if(-3<erro<3): valorI = (valorI+erro)*Ki
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        valorD = ((erro - erro0)*Kd)/tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel+valorPID)
        motorB.run(vel-valorPID)

        if(cronometro.time()>tempo): break
    motorC.hold()
    motorB.hold()

def segue_linha_c1_buraco(vel):
    valorLum = 35 #medir na linha toda vez
    Kp = 3.5
    Ki = 0.05 
    Kd = 10

    erro = 0
    valorI = 0
    t = 0

    cores = []
    cores_validas = [Color.YELLOW,Color.BLUE]

    cronometro.reset()
    while True:
        erro0 = erro  
        erro = valorLum - sensorc1.reflection()  
        valorP = erro*Kp
        if(-3<erro<3): valorI = (valorI+erro)*Ki
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        valorD = ((erro - erro0)*Kd)/tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel+valorPID)
        motorB.run(vel-valorPID)

        if(sensorc2.color() not in cores and sensorc2.color() in cores_validas):
            cores.append(sensorc2.color())
            cores_validas.remove(sensorc2.color())

        if(sensorc2.color()==None): break
    motorC.hold()
    motorB.hold()

    return cores

def segue_linha_c2_buraco(vel,array):
    valorLum = 35 #medir na linha toda vez
    Kp = 3.5
    Ki = 0.05 
    Kd = 10

    erro = 0
    valorI = 0
    t = 0

    cores_validas = [Color.YELLOW,Color.BLUE,Color.RED]
    print(array)
    cronometro.reset()
    while True:
        erro0 = erro  
        erro = valorLum - sensorc2.reflection()  
        valorP = erro*Kp
        if(-3<erro<3): valorI = (valorI+erro)*Ki
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        valorD = ((erro - erro0)*Kd)/tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel-valorPID)
        motorB.run(vel+valorPID)

        if(sensorc1.color() not in array and sensorc1.color() in cores_validas):
            array.append(sensorc1.color())
            break

    motorC.hold()
    motorB.hold()

    return array

def curva_1roda(time,modo):

    cronometro.reset()
    while(cronometro.time()<time):
        vel = -(cronometro.time()*10/time)**2+cronometro.time()*(200/time)+20
        if(modo==1):
            motorB.hold()
            motorC.dc(vel)
        if(modo==2):
            motorC.hold()
            motorB.dc(vel)

    motorB.brake()
    motorC.brake()

def dc_acel(time,modo,brk,re): #modo 1 termina vel 0 modo 2 vel max modo 3 começa vel max termina vel 0

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

            if not re:
                motorB.dc(vel+correcao)
                motorC.dc(vel-correcao)
            else:
                motorB.dc((-1)*vel+correcao)
                motorC.dc((-1)*vel-correcao)
                
            if brk: 
                if(sensorc1.reflection()<reflex_saida or sensorc2.reflection()<reflex_saida): break

        motorB.brake()
        motorC.brake()

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

            if not re:
                motorB.dc(vel+correcao)
                motorC.dc(vel-correcao)
            else:
                motorB.dc((-1)*vel+correcao)
                motorC.dc((-1)*vel-correcao)

            if brk: 
                if(sensorc1.reflection()<reflex_saida or sensorc2.reflection()<reflex_saida): break

        motorB.brake()
        motorC.brake()

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

            if not re:
                motorB.dc(vel+correcao)
                motorC.dc(vel-correcao)
            else:
                motorB.dc((-1)*vel+correcao)
                motorC.dc((-1)*vel-correcao)

            if brk: 
                if(sensorc1.reflection()<reflex_saida or sensorc2.reflection()<reflex_saida): break

        motorB.hold()
        motorC.hold()

def curva(angulo): #angulo positivo: direita, negativo: esquerda
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    Kp = 4
    Ki = 0.5
    Kd = 10
    
    t = 0
    integ = 0
    erro = 0
    while(motorB.angle()<angulo-20 or motorB.angle()>angulo+20):
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
        vel = 20 + correcao
        if(vel<0):
            if(vel>-20): vel = -20
        else:
            if(vel<20): vel = 20
        motorC.run(-vel)
        motorB.run(vel)

    motorC.hold()
    motorB.hold()

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

distRodas = 15.3
diamRodas = 5.5
rotacao = ((distRodas)/diamRodas)*360
vel=100

# dc_acel(5000,2,0,0)
# cronometro.reset()
# time=800
# while(cronometro.time()<time):
#     velB = (cronometro.time()*10/time)**2-cronometro.time()*(200/time)+20
#     if(velB<20): velB=20
#     motorB.dc(velB)
#     motorC.dc(vel)
#     if(sensorc1.reflection()<reflex_saida or sensorc2.reflection()<reflex_saida): break
# motorC.brake()
# motorB.brake()
# dc_acel(10000,3,1,0)
# motorB.dc(0)
# motorB.dc(0)

forward_while_same_reflection(50,50,10)
location = check_land_position_by_color(sensorc1,sensorc2)

while True:
    if(location=="RAMP"):

        print("rampa")
        dc_acel(500,3,0,1)
        curva(rotacao/2-50)
        forward_while_same_reflection(50,50,10)
        location = check_land_position_by_color(sensorc1,sensorc2)

    elif(location=="COLOR"):

        print("cor")
        curva_1roda(800,1)
        segue_linha_c1(100,1500)
        ordem_cores = segue_linha_c1_buraco(300)
        print(ordem_cores)
        if(len(ordem_cores)<2):
            dc_acel(500,3,0,1)
            curva(rotacao/2-50)
            ordem_cores = segue_linha_c2_buraco(300,ordem_cores)
            print(ordem_cores)
            dc_acel(500,3,0,1)
            curva(rotacao/2-70)
            segue_linha_c1_buraco(300)
        break

    elif(location=="EDGE"):
        
        print("buraco")
        dc_acel(500,3,0,1)
        curva(-rotacao/4)
        forward_while_same_reflection(50,50,10)
        location = check_land_position_by_color(sensorc1,sensorc2)

