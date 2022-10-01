"""
Módulo central pra controle do Robô.

Devem estar nesse módulo:
    - Classe 'Robot', com métodos e atributos para controle geral no robô
    - Estruturas de dados auxiliares aplicáveis a "qualquer" tipo de robô

Não devem estar nesse módulo:
    - Código específico de algum problema/desafio
"""


from pybricks.ev3devices import InfraredSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port


class Robot:
    """
    Classe que representa um robô genérico.
    """

    def __init__(
        self,
        motor_r: Port,
        motor_l: Port,
        motor_claw: Port = None,
        infra_side: Port = None,
    ) -> None:
        self.brick = EV3Brick()
        self.motor_r = Motor(motor_r)
        self.motor_l = Motor(motor_l)
        if motor_claw is not None:
            self.motor_claw = Motor(motor_claw)
        if infra_side is not None:
            self.infra_side = InfraredSensor(infra_side)
