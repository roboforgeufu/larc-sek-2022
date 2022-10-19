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


def wait_button_pressed(ev3: EV3Brick, button: Button = Button.CENTER):
    """
    Trava execução até que o botão especificado seja pressionado.
    """
    ev3.speaker.beep(800)
    while True:
        if button in ev3.buttons.pressed():
            break
