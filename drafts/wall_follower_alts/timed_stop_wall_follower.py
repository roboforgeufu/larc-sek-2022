#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import InfraredSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import StopWatch

ev3 = EV3Brick()

motor_l = Motor(Port.B)
motor_r = Motor(Port.C)
motor_claw = Motor(Port.A)
infra_side = InfraredSensor(Port.S3)
infra_front = InfraredSensor(Port.S2)


cronometer = StopWatch()


def ev3_print(*args, **kwargs):
    ev3.screen.print(*args, **kwargs)
    print(*args, **kwargs)


TARGET_DISTANCE = 20
STOP_TIME_MS = 1000

while True:
    target_diff = infra_side.distance() - TARGET_DISTANCE
    # target_diff > 0: muito longe
    # target_diff < 0: muito perto

    if target_diff > 0:
        # muito longe
        # para o motor da direita por um tempinho
        ...
    elif target_diff < 0:
        # muito perto
        # para o motor da esquerda por um tempinho
        ...

    ev3_print(target_diff, cronometer.time())
