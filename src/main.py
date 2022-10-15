#!/usr/bin/env pybricks-micropython

"""
Módulo para centralização dos processos e gerência da estratégia geral.

Podem estar nesse módulo coisas como:
    - Código que controla a ordem que as rotinas serão executadas
    - Código para controle de fluxo geral do robô
    - Chamadas a rotinas mais específicas
    - Instanciação de estruturas de dados, classes, etc.
    - Códigos específicos de comunicação com o EV3 ou gerência de recursos de sistemas
        operacionais no geral
    - Várias funções "main" alternativas, inclusive para testes ou calibragem de motores/sensores

Não devem estar nesse módulo:
    - Definição de constantes ou variáveis globais
    - Chamadas de função fora de escopo da main do módulo
    - Execução de código manipulando motores ou sensores diretamente
"""

import time

from pybricks.messaging import (
    BluetoothMailboxClient,
    BluetoothMailboxServer,
    LogicMailbox,
    NumericMailbox,
)
from pybricks.parameters import Color, Port
from pybricks.tools import DataLog, wait

import constants as const
from domain.collect import align_duct_center, duct_ends, find_duct
from domain.gas_duct import (
    armagedon_the_end_of_times,
    check_hole,
    duct_deliver,
    duct_follower_turn_routine,
    duct_measure_hole,
    gas_duct_routine,
)
from domain.localization import (
    check_land_position_by_color,
    land_position_routine,
    water_position_routine,
)
from robot import Robot
from utils import (
    PIDValues,
    accurate_color,
    ev3_print,
    get_hostname,
    wait_button_pressed,
)


def water_main(katara: Robot):
    """Main da Katara"""
    ev3_print(get_hostname(), ev3=katara.brick)
    server = BluetoothMailboxServer()
    ev3_print("SERVER: waiting for connection...", ev3=katara.brick)
    server.wait_for_connection()
    ev3_print("SERVER: connected!", ev3=katara.brick)

    ev3_print("1", ev3=katara.brick)
    logic_mbox = LogicMailbox("start", server)
    ev3_print("2", ev3=katara.brick)
    logic_mbox.wait()
    ev3_print("3", ev3=katara.brick)
    start = logic_mbox.read()
    ev3_print(start, ev3=katara.brick)
    if start:
        water_position_routine(katara)
        logic_mbox.send(True)


def testing_comunications_locations():
    """Main de testes"""
    hostname = get_hostname()
    if hostname == "katara":
        water_main(
            Robot(
                wheel_diameter=const.WHEEL_DIAMETER,
                wheel_distance=const.WHEEL_DIST,
            )
        )
    elif hostname == "toph":
        land_main(
            Robot(
                wheel_diameter=const.WHEEL_DIAMETER,
                wheel_distance=const.WHEEL_DIST,
                motor_claw=Port.A,
                motor_r=Port.C,
                motor_l=Port.B,
                # ultra_front_l=Port.S3,
                ultra_front_r=Port.S4,
                color_l=Port.S1,
                color_r=Port.S2,
                turn_correction=0.9,
                debug=False
            )
        )


def testing_duct_turn():
    toph = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        motor_r=Port.C,
        motor_l=Port.B,
        # color_l=Port.S1,
        # color_r=Port.S2,
        infra_side=Port.S3,
        ultra_front_r=Port.S4,
    )

    toph.move_to_distance(distance=100, sensor=toph.ultra_front_r)
    toph.pid_turn(-90)
    while True:
        toph.wall_aligner(speed=20)
        boolean = toph.pid_wall_follower()
        if boolean:
            toph.walk_to_hole(mode=2)
            toph.pid_walk(vel=-50, cm=5)
            toph.pid_walk(vel=50, cm=15)
            toph.pid_turn(90)
            toph.pid_walk(vel=50, cm=20)
        else:
            toph.pid_turn(-90)


def testing_duct_measurement():
    toph = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        motor_r=Port.C,
        motor_l=Port.B,
        motor_claw=Port.A,
        motor_sensor=Port.D,
        color_l=Port.S1,
        color_r=Port.S2,
        infra_side=Port.S3,
        ultra_front=Port.S4,
        debug=True,
        turn_correction=const.TOPH_TURN_CORRECTION,
    )

    # ev3_print(get_hostname(), ev3=toph.brick)
    # client = BluetoothMailboxClient()
    # ev3_print("CLIENT: establishing connection...")
    # client.connect(const.SERVER)
    # ev3_print("CLIENT: connected!")

    # num_mbox = NumericMailbox("start", client)

    toph.min_aligner(toph.infra_side.distance)
    toph.pid_wall_follower()
    check_hole(toph)
    return None


def land_main(toph: Robot):

    # while True:
    #     toph.ev3_print(toph.color_l.rgb(),accurate_color(toph.color_l.rgb()),clear=True)

    """Main da Toph"""

    # conexao entre os bricks por bluetooth
    ev3_print(get_hostname(), ev3=toph.brick)
    client = BluetoothMailboxClient()
    ev3_print("CLIENT: establishing connection...")
    client.connect(const.SERVER)
    ev3_print("CLIENT: connected!")

    # espera a katara sair da meeting area 
    # antes de comecar a rotina de localizacao
    logic_mbox = LogicMailbox("start", client)
    logic_mbox.send(True)
    logic_mbox.wait()
    start = logic_mbox.read()
    ev3_print(start, ev3=toph.brick)

    if start:

        # algoritmo de localizacao terrestre
        land_position_routine(toph)

        # vai ao primeiro terço da primeira cor
        toph.pid_walk(cm=13,vel=-60)

        while True:

            # alinha com a linha preta e vai um pouco pra frente para estar em cima da cor
            toph.pid_turn(90)
            toph.pid_walk(cm=5, vel=-60)
            toph.forward_while_same_reflection()
            toph.pid_walk(cm=5, vel=60)
            time.sleep(0.2)

            # funcao find_duct retorna se algum duto foi encontrado 
            # e o tamanho do arco de circunferencia que este representa
            duct_found, arc_length = find_duct(toph)
            
            if not duct_found:

                # vai para o prox terço da cor
                toph.forward_while_same_reflection(speed_r=-60, speed_l=-60)
                toph.pid_turn(-90)
                toph.pid_walk(cm=26, vel=-60)

            # verifica se o duto é coletável
            if (
                (accurate_color(toph.color_l.rgb()) == Color.YELLOW and arc_length > 5)
                or (accurate_color(toph.color_l.rgb()) == Color.RED and arc_length > 10)
                or (accurate_color(toph.color_l.rgb()) == Color.BLUE and arc_length > 15)
            ):

                # recolhe o duto
                toph.pid_walk(cm=max(1, (duct_found / 10) - 8), vel=50)
                toph.min_aligner(toph.ultra_front_r.distance)
                toph.pid_walk(cm=5, vel=30)
                toph.off_motors()
                toph.motor_claw.reset_angle(0)
                toph.motor_claw.run_target(300, 300)
                
                # alinha com a linha preta e o buraco para deixar o duto numa posição padrão
                toph.forward_while_same_reflection(speed_r=-60, speed_l=-60)
                toph.pid_walk(cm=13, vel=-60)
                toph.pid_turn(-90)
                toph.forward_while_same_reflection()
                toph.pid_align()
                
                # deixa o duto a 40cm do buraco
                toph.pid_walk(cm=40, vel=-60)
                toph.pid_turn(-90)
                toph.motor_claw.run_target(300, -10)
                
                # alinha com o buraco restaurando a posicao inicial
                toph.pid_walk(cm=5, vel=-60)
                toph.pid_turn(180)
                toph.forward_while_same_reflection()
                toph.pid_walk(cm=5, vel=-60)
                toph.pid_turn(-90)
                toph.forward_while_same_reflection()
                
                break
        
        # dutos subsequentes (comunicação bluetooth)

        # num_mbox = NumericMailbox("start", client)
    
    

def test_hole_reading():
    toph = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        motor_r=Port.C,
        motor_l=Port.B,
        motor_claw=Port.A,
        motor_sensor=Port.D,
        color_l=Port.S1,
        color_r=Port.S2,
        infra_side=Port.S3,
        ultra_front=Port.S4,
        debug=True,
        turn_correction=const.TOPH_TURN_CORRECTION,
    )

    toph.motor_claw.run_target(300, 300)
    gas_duct_routine(toph, delivery=15)


if __name__ == "__main__":
    testing_comunications_locations()
