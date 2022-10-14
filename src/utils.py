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


def accurate_color(rgb_tuple):
    """
    Processamento de cor pra evitar os erros da leitura padrão.
    """
    if rgb_tuple[0] in range(0,10) and rgb_tuple[1] in range(20,30) and rgb_tuple[2] in range(15,65):
        return Color.BLUE
    if rgb_tuple[0] in range(0,5) and rgb_tuple[1] in range(15,30) and rgb_tuple[2] in range(0,5):
        return Color.GREEN
    if rgb_tuple[0] in range(10, 65) and rgb_tuple[1] in range(5, 15) and rgb_tuple[2] in range(0,10):
        return Color.RED
    if rgb_tuple[0] in range(60, 75) and rgb_tuple[1] in range(30, 65) and rgb_tuple[2] in range(5, 20):
        return Color.YELLOW
    if rgb_tuple[0] > 65 and rgb_tuple[1] > 65 and rgb_tuple[2] > 65:
        return Color.WHITE
    if rgb_tuple[0] in range(1,15) and rgb_tuple[1] in range(1,15) and rgb_tuple[2] in range(1,15): #linha
        return Color.BLACK
    if sum(rgb_tuple)<=3:
        return "None"


def wait_button_pressed(ev3: EV3Brick, button: Button = Button.CENTER):
    """
    Trava execução até que o botão especificado seja pressionado.
    """
    while True:
        if button in ev3.buttons.pressed():
            break
