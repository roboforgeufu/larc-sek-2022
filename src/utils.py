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


def normalize_color(color_value, max_value=65):
    """
    Normalização dos valores de cor lidos.
    """
    if isinstance(color_value, int):
        return color_value / max_value
    else:
        return (
            color_value[0] / max_value,
            color_value[1] / max_value,
            color_value[2] / max_value,
        )


def between(value, min_value, max_value):
    """
    Verifica se um valor está entre um intervalo.
    """
    error_margin = 0.33
    return (
        min_value - error_margin * min_value
        <= value
        <= max_value + error_margin * max_value
    )


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
        between(red_normalized_value, 0.11, 0.15)
        and between(green_normalized_value, 0.36, 0.44)
        and between(blue_normalized_value, 0.48, 0.72)
    ):
        return Color.BLUE
    elif (
        green_normalized_value != 0
        and red_normalized_value / green_normalized_value <= 0.5
        and blue_normalized_value / green_normalized_value <= 0.5
    ):
        return Color.GREEN
    elif (
        between(red_normalized_value, 0.71, 0.8)
        and between(green_normalized_value, 0.46, 0.5)
        and between(blue_normalized_value, 0.07, 0.14)
    ):
        return Color.YELLOW
    elif (
        red_normalized_value == 0
        and green_normalized_value == 0
        and blue_normalized_value == 0
    ):
        return "None"
    elif (
        between(red_normalized_value, 0.7, 1)
        and between(green_normalized_value, 0.7, 1)
        and between(blue_normalized_value, 0.7, 1)
    ):
        return Color.WHITE
    elif (
        between(red_normalized_value, 0.57, 0.73)
        and between(green_normalized_value, 0.05, 0.07)
        and between(blue_normalized_value, 0, 0.05)
    ):
        return Color.RED
    else:
        return Color.BLACK


def wait_button_pressed(ev3: EV3Brick, button: Button = Button.CENTER):
    """
    Trava execução até que o botão especificado seja pressionado.
    """
    while True:
        if button in ev3.buttons.pressed():
            break
