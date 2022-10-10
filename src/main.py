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
    NumericMailbox
)
from pybricks.parameters import Port

import constants as const
from domain.collect import align_duct_center, duct_ends
from domain.localization import (
    check_land_position_by_color,
    land_position_routine,
    water_position_routine,
)
from robot import Robot
from utils import ev3_print, get_hostname


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
        # water_position_routine(katara)
        logic_mbox.send(True)


def land_main(toph: Robot):
    """Main da Toph"""
    ev3_print(get_hostname(), ev3=toph.brick)
    client = BluetoothMailboxClient()
    ev3_print("CLIENT: establishing connection...")
    client.connect(const.SERVER)
    ev3_print("CLIENT: connected!")

    logic_mbox = LogicMailbox("start", client)
    ev3_print("1", ev3=toph.brick)
    logic_mbox.send(True)
    ev3_print("2", ev3=toph.brick)
    logic_mbox.wait()
    ev3_print("3", ev3=toph.brick)
    start = logic_mbox.read()
    ev3_print("4", ev3=toph.brick)
    ev3_print(start, ev3=toph.brick)
    ev3_print("5", ev3=toph.brick)
    if start:
        land_position_routine(toph)


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
                motor_r=Port.B,
                motor_l=Port.C,
                color_l=Port.S1,
                color_r=Port.S2,
                ultra_front_l=Port.S3,
                ultra_front_r=Port.S4,
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

    boolean = toph.pid_wall_follower()
    if boolean:
        toph.wall_turn()
        toph.pid_accelerated_walk(time=500,mode=2)
    else:
        toph.pid_turn(-90)
    toph.pid_wall_follower()

def testing_duct_measurement():
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

    # ev3_print(get_hostname(), ev3=toph.brick)
    # client = BluetoothMailboxClient()
    # ev3_print("CLIENT: establishing connection...")
    # client.connect(const.SERVER)
    # ev3_print("CLIENT: connected!")

    # num_mbox = NumericMailbox("start", client)

    toph.pid_wall_follower()
    toph.walk_to_hole(mode=1)
    toph.pid_walk(cm=5,vel=-60)
    toph.walk_to_hole(mode=2)
    measurement = toph.hole_measurement()
    print(measurement)
    if(measurement>12):
        print("15cm")
        # num_mbox.send(2)
    elif(measurement>17):
        print("20cm")
        # num_mbox.send(3)
    else:
        print("10cm")
        # num_mbox.send(1)
    toph.pid_walk(cm=10,vel=60)
    toph.wall_aligner()
    toph.pid_wall_follower()
    return None


def testing_duct_seek_routine():
    katara = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        motor_r=Port.C,
        motor_l=Port.B,
        color_l=Port.S1,
        color_r=Port.S2,
        ultra_front_l=Port.S3,
        ultra_front_r=Port.S4,
    )

    last_seen_us = "R"  # L ou R

    while True:
        ev3_print(katara.ultra_front_l.distance(), katara.ultra_front_r.distance())
        if (
            katara.ultra_front_l.distance() < 50
            and katara.ultra_front_r.distance() < 50
        ):
            break

        if katara.ultra_front_l.distance() < 300:
            last_seen_us = "L"
        if katara.ultra_front_r.distance() < 300:
            last_seen_us = "R"

        if (
            katara.ultra_front_l.distance() < 300
            and katara.ultra_front_r.distance() < 300
        ):
            katara.motor_r.dc(50)
            katara.motor_l.dc(50)
        elif last_seen_us == "R":
            katara.motor_l.dc(50)
            katara.motor_r.dc(0)
        else:
            katara.motor_r.dc(50)
            katara.motor_l.dc(0)

    katara.off_motors()


if __name__ == "__main__":

    testing_duct_turn()
