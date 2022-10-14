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
from pybricks.tools import DataLog

import constants as const
from domain.collect import align_duct_center, duct_ends, find_duct
from domain.gas_duct import (
    armagedon_the_end_of_times,
    check_hole,
    duct_follower_turn_routine,
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


def testing_duct_seek_routine():
    katara = Robot(
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
        debug=True
    )

    # while True:
    #     katara.ev3_print(katara.color_l.rgb(),accurate_color(katara.color_l.rgb()),clear=True)

    # color_order = []
    # print(katara.pid_line_follower_color_id(vel=80,sensor=katara.color_r,array=color_order))
    # wait_button_pressed(katara.brick)

    land_position_routine(katara)
    
    katara.pid_walk(cm=13,vel=-60)
    # wait_button_pressed(katara.brick)

    while True:

        katara.pid_turn(90)
        katara.pid_walk(cm=5, vel=-60)
        katara.forward_while_same_reflection()
        # wait_button_pressed(katara.brick)

        katara.pid_walk(cm=5, vel=60)
        time.sleep(0.2)

        duct_found, arc_length = find_duct(katara)
        # wait_button_pressed(katara.brick)

        if not duct_found:

            # vai para o prox terço da cor

            katara.forward_while_same_reflection(speed_r=-60, speed_l=-60)
            katara.pid_turn(-90)
            katara.pid_walk(cm=26, vel=-60)

        # verifica se o duto é coletável

        # print(arc_length,accurate_color(katara.color_l.rgb()))
        if (
            (accurate_color(katara.color_l.rgb()) == Color.YELLOW and arc_length > 5)
            or (accurate_color(katara.color_l.rgb()) == Color.RED and arc_length > 10)
            or (accurate_color(katara.color_l.rgb()) == Color.BLUE and arc_length > 15)
        ):

            # recolhe o duto

            katara.pid_walk(cm=max(1, (duct_found / 10) - 8), vel=50)
            # wait_button_pressed(katara.brick)

            katara.min_aligner(katara.ultra_front_r.distance)
            # wait_button_pressed(katara.brick)

            katara.pid_walk(cm=5, vel=30)
            # wait_button_pressed(katara.brick)

            katara.off_motors()
            katara.motor_claw.reset_angle(0)

            katara.motor_claw.run_target(300, 300)
            # wait_button_pressed(katara.brick)

            katara.forward_while_same_reflection(speed_r=-60, speed_l=-60)
            # wait_button_pressed(katara.brick)

            katara.pid_walk(cm=13, vel=-60)
            # wait_button_pressed(katara.brick)

            katara.pid_turn(-90)
            # wait_button_pressed(katara.brick)

            katara.forward_while_same_reflection()
            katara.pid_align()
            # wait_button_pressed(katara.brick)

            katara.pid_walk(cm=40, vel=-60)
            # wait_button_pressed(katara.brick)

            katara.pid_turn(-90)
            # wait_button_pressed(katara.brick)

            katara.motor_claw.run_target(300, -10)
            # wait_button_pressed(katara.brick)

            katara.pid_walk(cm=5, vel=-60)
            # wait_button_pressed(katara.brick)

            katara.pid_turn(180)
            # wait_button_pressed(katara.brick)

            katara.forward_while_same_reflection()
            # wait_button_pressed(katara.brick)

            katara.pid_walk(cm=5, vel=-60)
            # wait_button_pressed(katara.brick)

            katara.pid_turn(-90)
            # wait_button_pressed(katara.brick)

            katara.forward_while_same_reflection()
            # wait_button_pressed(katara.brick)
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
    toph.move_to_distance(70, sensor=toph.ultra_front)
    toph.motor_claw.run_target(100, 120)
    toph.move_to_distance(40, sensor=toph.ultra_front)
    toph.move_to_distance(100, sensor=toph.ultra_front)
    # gas_duct_routine(toph)


if __name__ == "__main__":
    test_hole_reading()
