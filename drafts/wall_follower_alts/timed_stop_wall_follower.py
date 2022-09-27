#!/usr/bin/env pybricks-micropython
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
    target_diff = infra_side.distance() - TARGET_DISTANCE
    # target_diff > 0: muito longe
    # target_diff < 0: muito perto

    acc_angle_left = motor_l.angle() - left_angle_monitor
    acc_angle_right = motor_r.angle() - right_angle_monitor
    motors_diff = acc_angle_left - acc_angle_right

    if infra_side.distance() > 80:
        break

    if target_diff > TARGET_DIFF_FAR and not stop_right_flag and side_counter < 10:
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
        ev3.speaker.beep()

    if cronometer.time() > ANGLE_RESET_TIME:
        right_angle_monitor = motor_r.angle()
        left_angle_monitor = motor_r.angle()

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
