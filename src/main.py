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
)
from pybricks.parameters import Port
from pybricks.tools import DataLog

import constants as const
from domain.collect import align_duct_center, duct_ends, find_duct
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


def testing_duct_seek_routine():
    katara = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        motor_r=Port.C,
        motor_l=Port.B,
        ultra_front_l=Port.S3,
        ultra_front_r=Port.S4,
    )
    katara.align_front_wall()
    katara.brick.speaker.beep()
    time.sleep(1)
    duct_ends(katara)
    katara.brick.speaker.beep()
    time.sleep(1)
    duct_ends(katara, dir_sign=-1)
    katara.brick.speaker.beep()

    katara.ultra_front_l.distance()
    katara.ultra_front_r.distance()
    return None

    max_degrees = 500
    distance_range = 300

    while True:
        if (
            katara.ultra_front_r.distance() < 80
            and katara.ultra_front_l.distance() < 80
        ):
            katara.off_motors()
            break

        if (
            katara.ultra_front_r.distance() < distance_range
            and katara.ultra_front_l.distance() < distance_range
        ):
            katara.motor_r.dc(30)
            katara.motor_l.dc(30)
            continue

        katara.off_motors()

        katara.motor_l.reset_angle(0)
        while (
            abs(katara.motor_l.angle()) < max_degrees
            and katara.ultra_front_l.distance() >= distance_range
        ):
            ev3_print(katara.ultra_front_l.distance(), ev3=katara.brick)
            katara.motor_l.dc(30)
        katara.off_motors()

        katara.motor_r.reset_angle(0)
        while (
            abs(katara.motor_r.angle()) < max_degrees
            and katara.ultra_front_r.distance() >= distance_range
        ):
            ev3_print(katara.ultra_front_r.distance(), ev3=katara.brick)
            katara.motor_r.dc(30)
        katara.off_motors()


if __name__ == "__main__":
    testing_duct_seek_routine()
