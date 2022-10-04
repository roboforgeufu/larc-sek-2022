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

from pybricks.parameters import Port

import utils
from robot import Robot


def testing():
    """Main de testes"""

    ev3_client = Robot(
        motor_r=Port.B,
        motor_l=Port.C,
        infra_side=Port.S3,
    )

    utils.ev3_print(utils.get_hostname(), ev3=ev3_client.brick)

    return 0


if __name__ == "__main__":
    testing()