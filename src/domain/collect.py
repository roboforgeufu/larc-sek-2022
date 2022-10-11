import time

from pybricks.parameters import Color
from pybricks.tools import wait

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
        robot.ultra_front_l.distance(silent=True)
    elif dir_sign == -1:
        sensor = robot.ultra_front_l
        robot.ultra_front_r.distance(silent=True)

    previous_distance = sensor.distance()
    while abs(previous_distance - sensor.distance()) < const.DUCT_ENDS_US_DIFF:
        previous_distance = sensor.distance()
        robot.motor_l.dc(dir_sign * speed)
        robot.motor_r.dc(dir_sign * -speed)
        ev3_print(
            sensor.distance(),
            ev3=robot.brick,
        )

    robot.off_motors()


def align_duct_center(robot: Robot):
    """O robô alinha no centro do duto correspondente"""

    first_dist = duct_ends(robot)
    ev3_print(
        robot.ultra_front_l.distance(),
        robot.ultra_front_r.distance(),
        ev3=robot.brick,
    )

    robot.simple_turn(15)
    second_dist = duct_ends(robot, dir_sign=-1)

    ev3_print(
        first_dist,
        second_dist,
        ev3=robot.brick,
    )

    avg = (first_dist + second_dist) / 2
    robot.move_to_distance(distance=avg)


def find_duct(robot: Robot):
    lowest_ultra_value = 255
    forward_velocity = max(60, 500 / robot.stopwatch.time())
    backward_velocity = min(-60, -500 / robot.stopwatch.time())

    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)

    robot.stopwatch.reset()
    while robot.stopwatch.time() < 300:
        wait(5)
        robot.motor_l.dc(forward_velocity)
        robot.motor_r.dc(backward_velocity)
        prev_lowest_ultra_value = lowest_ultra_value
        lowest_ultra_value = min(
            lowest_ultra_value,
            robot.ultra_front_l.distance(),
            robot.ultra_front_r.distance(),
        )
        if prev_lowest_ultra_value != lowest_ultra_value:
            escape_angle_l = robot.motor_l.angle()
            escape_angle_r = robot.motor_r.angle()

    robot.stopwatch.reset()
    while robot.stopwatch.time() < 600:
        wait(5)
        robot.motor_r.dc(forward_velocity)
        robot.motor_l.dc(backward_velocity)
        prev_lowest_ultra_value = lowest_ultra_value
        lowest_ultra_value = min(
            lowest_ultra_value,
            robot.ultra_front_l.distance(),
            robot.ultra_front_r.distance(),
        )
        if prev_lowest_ultra_value != lowest_ultra_value:
            escape_angle_l = robot.motor_l.angle()
            escape_angle_r = robot.motor_r.angle()

    robot.stopwatch.reset()
    while robot.stopwatch.time() < 300:
        wait(5)
        robot.motor_l.dc(forward_velocity)
        robot.motor_r.dc(backward_velocity)
        prev_lowest_ultra_value = lowest_ultra_value
        lowest_ultra_value = min(
            lowest_ultra_value,
            robot.ultra_front_l.distance(),
            robot.ultra_front_r.distance(),
        )
        if prev_lowest_ultra_value != lowest_ultra_value:
            escape_angle_l = robot.motor_l.angle()
            escape_angle_r = robot.motor_r.angle()

    target_angle_l = abs(robot.motor_l.angle()) + escape_angle_l
    target_angle_r = abs(robot.motor_r.angle()) + escape_angle_r
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)

    if robot.motor_l.angle() > escape_angle_l:
        while (
            abs(robot.motor_l.angle()) < target_angle_l
            or abs(robot.motor_r.angle()) < target_angle_r
        ):
            print(abs(robot.motor_l.angle()), abs(robot.motor_r.angle()))
            robot.motor_r.dc(30)
            robot.motor_l.dc(-30)
    else:
        while (
            abs(robot.motor_l.angle()) < target_angle_l
            or abs(robot.motor_r.angle()) < target_angle_r
        ):
            robot.motor_r.dc(-30)
            robot.motor_l.dc(30)

    robot.off_motors()
