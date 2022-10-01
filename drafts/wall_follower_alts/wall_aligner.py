#!/usr/bin/env pybricks-micropython

import time

from pybricks.ev3devices import InfraredSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port
from pybricks.tools import StopWatch

ev3 = EV3Brick()

motor_l = Motor(Port.B)
motor_r = Motor(Port.C)
motor_claw = Motor(Port.A)
infra_side = InfraredSensor(Port.S3)
# infra_front = InfraredSensor(Port.S2)


cronometer = StopWatch()


def ev3_print(*args, **kwargs):
    ev3.screen.print(*args, **kwargs)
    print(*args, **kwargs)


def hold_all(*motors: Motor):
    for motor in motors:
        motor.hold()


def wall_aligner(dc_speed: int = 30, max_angle: int = 300):
    # TODO: recebe "robot" por parametro
    hold_all(motor_l, motor_r)

    initial_angle_r = motor_r.angle()
    initial_angle_l = motor_l.angle()
    infra_reads = [infra_side.distance()]

    # primeira leitura inicial, só pra ter uma noção da situação
    while (
        abs(motor_r.angle() - initial_angle_r) <= 50
        and abs(motor_l.angle() - initial_angle_l) <= 50
    ):
        infra_reads.append(infra_side.distance())
        motor_r.dc(dc_speed)
        motor_l.dc(-dc_speed)
    motor_l.dc(0)
    motor_r.dc(0)

    if min(infra_reads) < infra_reads[0] and min(infra_reads) < infra_reads[-1]:
        # a menor leitura das iniciais já está entre a primeira e a última leitura
        # logo, já está suficientemente alinhado com a parede
        ev3.speaker.beep()
        return

    if infra_reads[0] - infra_reads[-1] >= 0:
        # leituras estão diminuindo
        direction_multiplier = 1
        ev3.light.on(Color.RED)
    else:
        # leituras estão aumentando, deve inverter a direção da curva
        direction_multiplier = -1
        ev3.light.on(Color.ORANGE)

    # if direction_multiplier == -1:
    while infra_side.distance() > min(infra_reads):
        motor_r.dc(dc_speed * direction_multiplier)
        motor_l.dc(-dc_speed * direction_multiplier)

    initial_angle_r = motor_r.angle()
    initial_angle_l = motor_l.angle()
    infra_reads.clear()

    while (
        abs(motor_r.angle() - initial_angle_r) <= max_angle
        and abs(motor_l.angle() - initial_angle_l) <= max_angle
    ):
        motor_r.dc(dc_speed * direction_multiplier)
        motor_l.dc(-dc_speed * direction_multiplier)

        infra_reads.append(infra_side.distance())

        ev3_print(infra_reads[-1])
        if len(infra_reads) > 2 and infra_reads[-1] > infra_reads[-2]:
            # última leitura é maior que a penúltima
            break
    hold_all(motor_r, motor_l)


while True:
    wall_aligner()
    time.sleep(1)
