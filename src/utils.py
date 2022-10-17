"""
Módulo de utilidades gerais de código.
"""

import os

from pybricks.hubs import EV3Brick
from pybricks.parameters import Button, Color


class PIDValues:  # pylint: disable=too-few-public-methods
    """Variáveis de controle PID."""

    def __init__(  # pylint: disable=invalid-name
        self,
        kp: float = 0,
        ki: float = 0,
        kd: float = 0,
        target=None,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target


def ev3_print(*args, ev3: EV3Brick = None, **kwargs):
    """
    Função para logs.
    Imprime os valores tanto na tela do EV3 (caso disponível) quanto na do terminal.
    """
    if ev3 is not None:
        ev3.screen.print(*args, **kwargs)
    print(*args, **kwargs)


def get_hostname() -> str:
    """
    Retorna o hostname do dispositivo. Feito pensando em verificar o nome do BRICK.
    """
    stream = os.popen("hostname")  # nosec
    return stream.read().split()[0]


def normalize_color(color_value):
    return color_value/255

def accurate_color(rgb_tuple):
    """
    Processamento de cor pra evitar os erros da leitura padrão.
    """
    red_value = rgb_tuple[0]
    green_value = rgb_tuple[1]
    blue_value = rgb_tuple[2]

    red_normalized_value = normalize_color(red_value)
    green_normalized_value = normalize_color(green_value)
    blue_normalized_value = normalize_color(blue_value)

    
    if (
        red_value in range(0, 10)
        and green_value in range(20, 30)
        and blue_value in range(15, 65)
    )
        return Color.BLUE
    if (
        red_value in range(0, 5)
        and green_value in range(7, 30)
        and blue_value in range(0, 5)
    ):
        return Color.GREEN
    if (
        red_value in range(10, 65)
        and green_value in range(5, 15)
        and blue_value in range(0, 10)
    ):
        return Color.RED
    if (
        red_value in range(60, 75)
        and green_value in range(30, 65)
        and blue_value in range(5, 20)
    ):
        return Color.YELLOW
    if red_value > 65 and green_value > 65 and blue_value > 65:
        return Color.WHITE
    if (
        red_value in range(1, 15)
        and green_value in range(1, 15)
        and blue_value in range(1, 15)
    ):  # linha
        return Color.BLACK
    if sum(rgb_tuple) <= 3:
        return "None"
    return Color.BLACK


def wait_button_pressed(ev3: EV3Brick, button: Button = Button.CENTER):
    """
    Trava execução até que o botão especificado seja pressionado.
    """
    while True:
        if button in ev3.buttons.pressed():
            break
