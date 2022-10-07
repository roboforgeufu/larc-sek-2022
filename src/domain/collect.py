from pybricks.parameters import Color

import constants as const
from robot import Robot
from utils import accurate_color, ev3_print


def duct_ends(
    robot: Robot,
    speed: int = 20,
    dir_sign: int = 1,  # 1 ou -1
):
    """
    Com os dois ultrassonicos da frente apontando para um duto,
    o robô gira até estar de frente para a ponta do tubo.
    - `dir_sign` é 1 ou -1 e representa a direção da curva.
    """
    if dir_sign == 1:
        sensor = robot.ultra_front_r
    elif dir_sign == -1:
        sensor = robot.ultra_front_l

    initial_value = sensor.distance()
    while abs(initial_value - sensor.distance()) < const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(dir_sign * speed)
        robot.motor_r.dc(dir_sign * -speed)
    robot.off_motors()
    return min(robot.ultra_front_l.distance(), robot.ultra_front_r.distance())


def align_duct_center(robot: Robot):
    """O robô alinha no centro do duto correspondente"""
    first_dist = duct_ends(robot)

    robot.simple_turn(15)
    second_dist = duct_ends(robot, dir_sign=-1)

    avg = (first_dist + second_dist) / 2

    robot.move_to_distance(avg, robot.ultra_front_r, turning=0.3)

    wheel_dg_i = robot.motor_r.angle()

    robot.simple_turn(15, speed=-30)
    duct_ends(robot)

    robot.move_to_distance(avg, robot.ultra_front_l, turning=-0.3)

    wheel_dg_f = robot.motor_r.angle()
    wheel_dg = (wheel_dg_f - wheel_dg_i) / 2

    while abs(wheel_dg - robot.motor_r.angle()) > 10:
        robot.motor_l.dc(30)
        robot.motor_r.dc(-30)
    robot.off_motors()
