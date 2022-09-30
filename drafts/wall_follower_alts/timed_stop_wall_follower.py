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


TARGET_DISTANCE = 20
STOP_TIME_MS = 100
ANGLE_RESET_TIME = STOP_TIME_MS * 8

TARGET_DIFF_NEAR = -2
TARGET_DIFF_FAR = 2

DC_SPEED = 30

MOTOR_DIFF_CEIL = 350


stop_left_flag = False
stop_right_flag = False
right_angle_monitor = motor_r.angle()
left_angle_monitor = motor_l.angle()
side_counter = 0

while True:
    if side_counter >= 8:
        # alinha na parede caso tenha tentado muitas vezes ir pra direita
        ev3.speaker.beep(frequency=100)
        ev3.light.on(Color.GREEN)

        wall_aligner()

        ev3.light.off()
        ev3.speaker.beep()
        side_counter = 0

    target_diff = infra_side.distance() - TARGET_DISTANCE
    # target_diff > 0: muito longe
    # target_diff < 0: muito perto

    acc_angle_left = motor_l.angle() - left_angle_monitor
    acc_angle_right = motor_r.angle() - right_angle_monitor
    motors_diff = acc_angle_left - acc_angle_right

    if target_diff > TARGET_DIFF_FAR and not stop_right_flag:
        # muito longe
        # para o motor da direita por um tempinho
        stop_right_flag = True
        cronometer.reset()
        ev3.light.on(Color.ORANGE)
        side_counter += 1

    elif target_diff < TARGET_DIFF_NEAR and not stop_left_flag:
        # muito perto
        # para o motor da esquerda por um tempinho
        stop_left_flag = True
        cronometer.reset()
        ev3.light.on(Color.RED)
        side_counter -= 1

    # Reset das flags de acordo com o tempo
    if any([stop_right_flag, stop_left_flag]) and cronometer.time() > STOP_TIME_MS:
        stop_left_flag = stop_right_flag = False
        ev3.light.off()

    if cronometer.time() > ANGLE_RESET_TIME:
        right_angle_monitor = motor_r.angle()
        left_angle_monitor = motor_r.angle()
        if side_counter < 0:
            side_counter = 0

    # Controle de velocidade (a partir das flags)
    speed_left = speed_right = DC_SPEED
    if stop_left_flag:
        speed_left = 0
    elif stop_right_flag:
        speed_right = 0

    # Atribuição de velocidade
    motor_l.dc(speed_left)
    motor_r.dc(speed_right)

    # -- LOGGING ---
    # ev3_print(target_diff, cronometer.time(), stop_left_flag, stop_right_flag)
    ev3_print(side_counter)


motor_l.hold()
motor_r.hold()
