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
from domain.collect import (
    align_duct_center,
    duct_ends,
    duct_seek_routine,
    duct_seek_routine_new,
)
from domain.gas_duct import (
    armagedon_the_end_of_times,
    check_hole,
    duct_deliver,
    duct_follower_turn_routine,
    duct_get,
    duct_measure_hole,
    gas_duct_routine,
)
from domain.localization import (
    back_from_water_routine,
    back_to_water_routine,
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
    normalize_color,
    wait_button_pressed,
)


def main():
    """Main"""
    hostname = get_hostname()
    if hostname == "katara":
        water_main(
            Robot(
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
                turn_correction=const.KATARA_TURN_CORRECTION,
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
                ultra_front=Port.S4,
                infra_side=Port.S3,
                color_l=Port.S1,
                color_r=Port.S2,
                turn_correction=0.9,
                debug=True,
            )
        )


def land_main(toph: Robot):
    """Main da Toph"""

    # # conexao entre os bricks por bluetooth
    # toph.ev3_print(get_hostname())
    # server = BluetoothMailboxServer()
    # toph.ev3_print("SERVER: waiting for connection...")
    # server.wait_for_connection()
    # toph.ev3_print("SERVER: connected!")

    # # espera a katara sair da meeting area
    # # antes de comecar a rotina de localizacao
    # logic_mbox = LogicMailbox("start", server)

    # # espera a katara falar q conectou
    # logic_mbox.wait()
    # logic_mbox.send(True)

    # # katara desceu a rampa
    # logic_mbox.wait()

    #duct_seek_routine_new(toph,Color.RED)
    #wait_button_pressed(toph.brick)

    # algoritmo de localizacao terrestre
    color_order = land_position_routine(toph)
    valid_colors = [Color.YELLOW, Color.RED, Color.BLUE]
    for color in valid_colors:
        if color not in color_order:
            color_order.append(color)
    ev3_print(color_order)
    # termina com o sensor no buraco na primeira cor da esquerda p/ a direita

    # manobras
    toph.pid_walk(cm=13, vel=-60)
    toph.pid_turn(90)
    toph.pid_walk(cm=10, vel=-60)
    toph.forward_while_same_reflection()
    toph.pid_walk(cm=7, vel=-60)
    toph.one_wheel_turn(700, toph.motor_l)
    # termina com o sensor esquerdo sobre a linha preta da primeira cor

    # dutos subsequentes (comunicação bluetooth)

    # num_mbox = NumericMailbox("measures", server)
    while True:

        # num_mbox.wait()
        # num = num_mbox.read()
        num = 10

        if num == 10:
            toph.pid_line_grabber(100, 2000, toph.color_l)
            toph.pid_line_follower_color_id(80, toph.color_l, break_color=Color.YELLOW)
            index = color_order.index(Color.YELLOW)
        if num == 15:
            toph.pid_line_grabber(100, 2000, toph.color_l)
            toph.pid_line_follower_color_id(80, toph.color_l, break_color=Color.RED)
            index = color_order.index(Color.RED)
        if num == 20:
            toph.pid_line_grabber(100, 2000, toph.color_l)
            toph.pid_line_follower_color_id(80, toph.color_l, break_color=Color.BLUE)
            index = color_order.index(Color.BLUE)

        color_order.append(None)
        duct_seek_routine_new(toph,color_order[index+1])


def water_main(katara: Robot):
    """Main da Katara"""
    # conexao entre os bricks por bluetooth
    katara.ev3_print(get_hostname())
    client = BluetoothMailboxClient()
    ev3_print("CLIENT: establishing connection...")
    client.connect(const.SERVER)
    ev3_print("CLIENT: connected!")

    numeric_mbox = NumericMailbox("measures", client)

    katara.ev3_print("1")
    logic_mbox = LogicMailbox("start", client)
    katara.ev3_print("2")
    # Confirma conexão (sync)
    logic_mbox.send(True)
    katara.ev3_print("3")
    # Espera confirmação da Toph
    logic_mbox.wait()

    water_position_routine(katara)

    # Avisa toph que está fora da meeting area
    logic_mbox.send(True)

    measured_value = gas_duct_routine(katara)
    numeric_mbox.send(measured_value)


def test_katara():
    katara = Robot(
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
        turn_correction=const.KATARA_TURN_CORRECTION,
    )

    katara.motor_claw.run_target(300, 300)
    wait_button_pressed(katara.brick)

    water_position_routine(katara)
    wait_button_pressed(katara.brick)

    gas_duct_routine(katara)
    wait_button_pressed(katara.brick)

    back_from_water_routine(katara)
    wait_button_pressed(katara.brick)

    duct_get(katara)

    wait_button_pressed(katara.brick)
    back_to_water_routine(katara)


def white_calibration():
    robot = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        color_l=Port.S1,
        color_r=Port.S2,
        debug=True,
    )

    robot.brick.speaker.beep()
    color = ["BRANCO"]
    for c in color:
        robot.ev3_print(c)
        color_array = []

        logger = DataLog("WHITE_MAX_VALUE", name="log_" + c)
        wait_button_pressed(robot.brick)

        for _ in range(200):
            color_array.append(robot.color_l.rgb())
            color_array.append(robot.color_r.rgb())
            wait(10)
        robot.brick.speaker.beep()
        # save color array to file
        x_max = max([x for x, y, z in color_array])
        y_max = max([y for x, y, z in color_array])
        z_max = max([z for x, y, z in color_array])

        # get the highest value between the maxes
        max_value = max(x_max, y_max, z_max)
        logger.log(max_value)
        return max_value


def color_calibration():
    robot = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        color_l=Port.S1,
        color_r=Port.S2,
        debug=True,
    )

    robot.brick.speaker.beep()
    max_value = white_calibration()
    color = ["BRANCO", "PRETO", "VERDE", "AZUL", "VERMELHO", "AMARELO", "BURACO"]
    for c in color:
        robot.ev3_print(c)
        logger = DataLog("rgb_left", "rgb_right", name="log_" + c)

        wait_button_pressed(robot.brick)
        for _ in range(200):
            logger.log(
                normalize_color(robot.color_l.rgb(), max_value),
                normalize_color(robot.color_r.rgb(), max_value),
            )
            wait(10)
        robot.brick.speaker.beep()


def color_guessing():
    # while a button is not pressed, read the color and print it on the screen
    robot = Robot(
        wheel_diameter=const.WHEEL_DIAMETER,
        wheel_distance=const.WHEEL_DIST,
        color_l=Port.S1,
        color_r=Port.S2,
        debug=True,
    )

    robot.brick.speaker.beep()
    while not robot.brick.buttons.pressed():
        robot.ev3_print(accurate_color(robot.color_l.rgb()))
        robot.ev3_print(accurate_color(robot.color_r.rgb()))
        wait(100)


if __name__ == "__main__":
    test_katara()
