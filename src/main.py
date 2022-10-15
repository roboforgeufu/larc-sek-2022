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
                ultra_front_r=Port.S4,
                color_l=Port.S1,
                color_r=Port.S2,
                turn_correction=0.9,
                debug=False,
            )
        )


def land_main(toph: Robot):

    # while True:
    #     toph.ev3_print(toph.color_l.rgb(),accurate_color(toph.color_l.rgb()),clear=True)

    """Main da Toph"""

    toph.ev3_print(get_hostname)
    server = BluetoothMailboxServer()
    toph.ev3_print("SERVER: waiting for connection...")
    server.wait_for_connection()
    toph.ev3_print("SERVER: connected!")

    # espera a katara sair da meeting area
    # antes de comecar a rotina de localizacao
    logic_mbox = LogicMailbox("start", server)
    logic_mbox.send(True)
    logic_mbox.wait()
    start = logic_mbox.read()
    toph.ev3_print(start)

    if start:

        # algoritmo de localizacao terrestre
        land_position_routine(toph)

        # vai ao primeiro terço da primeira cor
        toph.pid_walk(cm=13, vel=-60)

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
                or (
                    accurate_color(toph.color_l.rgb()) == Color.BLUE and arc_length > 15
                )
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


def water_main(katara: Robot):
    """Main da Katara"""
    # conexao entre os bricks por bluetooth
    katara.ev3_print(get_hostname())
    client = BluetoothMailboxClient()
    ev3_print("CLIENT: establishing connection...")
    client.connect(const.SERVER)
    ev3_print("CLIENT: connected!")

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

    gas_duct_routine(katara)


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

    water_position_routine(katara)
    gas_duct_routine(katara)


if __name__ == "__main__":
    test_katara()
