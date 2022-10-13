"""
Centralização de definição de constantes.

Devem estar nesse módulo:
    - Definição de variáveis constantes (e que obviamente não devem ser alteradas em execução)
    - Comentários sobre o que significam as constantes
    - Constantes com nomes bem significativos

Não devem estar nesse módulo:
    - Qualquer tipo de código além de declaração de constantes
    - Variáveis globais utilizadas para controle de algoritmos (que sofrem alterações em execução)
"""

# Dimensões do Robô
WHEEL_DIAMETER = 5.5
WHEEL_DIST = 15.3

# Comunicação
SERVER = "katara"

PID_TURN_ACCEPTABLE_DEGREES_PERC = 0.9


# Coleta

DUCT_ENDS_US_DIFF = 200


# Seguidor de parede
WALL_FOLLOWER_FRONT_DIST = 110
