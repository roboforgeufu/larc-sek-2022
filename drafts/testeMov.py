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

def curva(angulo): #angulo positivo: direita, negativo: esquerda
    motorB.reset_angle(0)
    motorC.reset_angle(0)
    Kp = 4
    Ki = 0.5
    Kd = 10
    
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

def walk(aFuncao=0, bFuncao=0, cFuncao=0, graus=0, intervOscilacao=0, insideReset=True):
        """Anda com o robo."""
        # TODO: Deixar velocidade constante.
        print("Walking...")
        if insideReset:
            motorC.reset_angle(0)
            motorB.reset_angle(0)
        velocDir = 0
        velocEsq = 0
        while True:
            # print(motorC.angle(), motorB.angle())
            # Funcao que descreve a velocidade em funcao da porcentagem do deslocamento ja realizado
            percDeslocado = (motorC.angle() / graus) * 100
            velocEsq = (aFuncao * (percDeslocado ** 2) + bFuncao * percDeslocado + cFuncao) * 10
            velocDir = velocEsq

            # Teto
            if velocEsq > 900:
                velocEsq = 900
            if velocDir > 900:
                velocDir = 900

            diferenca_EsqDir = abs(motorC.angle()) - abs(motorB.angle())
            if abs(diferenca_EsqDir) > 3:
                if diferenca_EsqDir > 0:
                    # motorC andou mais, joga mais velocidade no motorB
                    velocEsq = velocEsq * (1 - (intervOscilacao / 100))
                    velocDir = velocDir * (1 + (intervOscilacao / 100))
                else:
                    # motorB andou mais, joga mais velocidade no motorC
                    velocEsq = velocEsq * (1 + (intervOscilacao / 100))
                    velocDir = velocDir * (1 - (intervOscilacao / 100))
            motorC.run(velocEsq)
            motorB.run(velocDir)
            if (abs(velocEsq) < 50 and abs(velocDir) < 50) or percDeslocado >= 99:
                break

def turn(aFuncao, bFuncao, cFuncao, grausCurva, fix=True):
        print("Turning...")
        self.lmotor.reset_angle(0)
        self.rmotor.reset_angle(0)

        # P/ calibrar
        K = const.K
        grausMotor = int(K * grausCurva / 90)

        mediaPercorrida = 0
        while mediaPercorrida < 99:
            # Calculo da velocidade de ambos os motores em funcao da media do deslocamento
            # percorrido por cada um
            percPercorridoEsq = (abs(self.lmotor.angle()) / grausMotor) * 100
            percPercorridoDir = (abs(self.rmotor.angle()) / grausMotor) * 100
            mediaPercorrida = (percPercorridoDir + percPercorridoEsq) / 2.0

            velocidade = (
                aFuncao * (mediaPercorrida ** 2) + bFuncao * mediaPercorrida + cFuncao
            ) * 10

            self.lmotor.run(velocidade)
            self.rmotor.run(-velocidade)
        self.lmotor.stop()
        self.rmotor.stop()

        if fix:
            while True:
                # Para o motor Esquerdo:
                difEsq = abs(self.lmotor.angle()) - grausMotor
                #print("DifEsq >>",difEsq, ">>", self.lmotor.angle(), grausMotor)
                if abs(difEsq) > 3:
                    # A diferenca eh consideravel
                    if difEsq > 0:
                        # Andou mais do que devia
                        sinalEsq = -1 *(
                            self.lmotor.angle() / abs(self.lmotor.angle())
                        )  # Deve se movimentar no sentido contrario ao mov anterior
                    else:
                        # Andou menos do que devia
                        sinalEsq = self.lmotor.angle() / abs(
                            self.lmotor.angle()
                        )  # Deve se movimentar no mesmo sentido do movimento anterior
                    velocEsq = sinalEsq * (2 * abs(difEsq) + 50)
                else:
                    # A diferenca eh irrelevante
                    velocEsq = 0

                # Para o motor Direito:
                difDir = abs(self.rmotor.angle()) - grausMotor
                #print("DifDir >>", difDir, ">>", self.rmotor.angle(), grausMotor)
                if abs(difDir) > 3:
                    # A diferenca eh consideravel
                    if difDir > 0:
                        # Andou mais do que devia
                        sinalDir = -1 * (
                            self.rmotor.angle() / abs(self.rmotor.angle())
                        )  # Deve se movimentar no sentido contrario ao mov anterior
                    else:
                        # Andou menos do que devia
                        sinalDir = self.rmotor.angle() / abs(
                            self.rmotor.angle()
                        )  # Deve se movimentar no mesmo sentido do movimento anterior
                    velocDir = sinalDir * (2 * abs(difDir) + 50)
                else:
                    # A diferenca eh irrelevante
                    velocDir = 0

                # Por fim, manda a velocidade calculada para os motores
                self.lmotor.run(velocEsq)
                self.rmotor.run(velocDir)

                # Se qualquer dos motores estiver fora do intervalo
                # seguro, entao ele ainda nao comeca a contar
                if (difDir not in range(-3, 4)) or (difEsq not in range(-3, 4)):
                    self.stopwatch.reset()  # Nao comecar a contar <=> resetar constantemente o self.stopwatch

                #print("Tempo:", self.stopwatch.time(), "\ Vel:", velocDir, velocEsq)
                #print(difDir, difEsq)
                # Caso passem 400 ms sem resetar o self.stopwatch, ou seja, 300 ms com ambos os motores na zona segura
                if self.stopwatch.time() > 100:
                    print("E:", self.lmotor.angle(), "D:", self.rmotor.angle())
                    print(velocEsq, velocDir)
                    print()
                    break

def sobe_garra(): 
    cronometro.reset()
    while(cronometro.time()<400): motorA.dc(100)
    motorA.hold()
    wait(500)

def desce_garra(): 
    cronometro.reset()
    while(cronometro.time()<300): motorA.dc(-100)
    motorA.hold()
    wait(500)

#anda_reto_graus(300,2000,2)
walk(aFuncao = -0.01, bFuncao = 1, cFuncao=40, graus=2000)