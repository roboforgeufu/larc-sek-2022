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
import math

# Dimensões do Robô
WHEEL_DIAMETER = 5.5
WHEEL_DIST = 15.3
WHEEL_LENGTH = WHEEL_DIAMETER * math.pi  # 360 graus = 1 rotacao; 1 rotacao = 17.3cm

# Comunicação
SERVER = "toph"

# Localização
OBSTACLE_DIST = 90

# Curvas
PID_TURN_ACCEPTABLE_DEGREE_DIFF = 3
PID_TURN_MIN_SPEED = 5
MIN_DEGREES_CURVE_THRESHOLD = 30

TESTING_TOPH_TURN_CORRECTION = 0.9
KATARA_TURN_CORRECTION = 0.9
TOPH_TURN_CORRECTION = 0.9


# Coleta

DUCT_ENDS_US_DIFF = 200
DIST_LINE_TO_END = 250

# Seguidor de parede
WALL_FOLLOWER_FRONT_DIST = 120
WALL_FOLLOWER_SIDE_DIST = 9
WALL_SEEN_DIST = 10
SEARCH_WALL_SPEED = 50

COL_REFLECTION_HOLE_MAX = 3
COL_REFLECTION_HOLE_DIFF = 4

# Motor infravermelho
INFRA_UP = 0
INFRA_DOWN = 550
INFRA_SPEED = 300

# Garra
CLAW_UP = 290
CLAW_DOWN = 0
