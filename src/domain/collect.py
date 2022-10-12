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

    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)

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

    return (abs(robot.motor_l.angle()) + abs(robot.motor_l.angle()))/2


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
    lowest_ultra_value_l = 2550
    lowest_ultra_value_r = 2550
    forward_velocity = max(60, 500 / robot.stopwatch.time())
    backward_velocity = min(-60, -500 / robot.stopwatch.time())

    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)

    times = [400,800,400]
    i=0
    while(i!=3):
        robot.stopwatch.reset()
        while robot.stopwatch.time() < times[i]:
            wait(5)
            sign = -1 if i % 2 == 0 else 1
            robot.motor_l.dc(sign * forward_velocity)
            robot.motor_r.dc(sign * backward_velocity)
            prev_lowest_ultra_value_l = lowest_ultra_value_l
            prev_lowest_ultra_value_r = lowest_ultra_value_r
            lowest_ultra_value_l = min(
                lowest_ultra_value_l,
                robot.ultra_front_l.distance(),
            )
            lowest_ultra_value_r = min(
                lowest_ultra_value_r,
                robot.ultra_front_r.distance(),
            )

            if prev_lowest_ultra_value_l != lowest_ultra_value_l:
                escape_angle_l = robot.motor_l.angle()

            if prev_lowest_ultra_value_r != lowest_ultra_value_r:
                escape_angle_r = robot.motor_r.angle()
        i+=1

    target_angle_l = abs(robot.motor_l.angle()) + escape_angle_l
    final_angle_r = robot.motor_r.angle()
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)

    if robot.motor_l.angle() > escape_angle_l:
        while (
            abs(robot.motor_l.angle()) < target_angle_l
        ):
            robot.motor_r.dc(30)
            robot.motor_l.dc(-30)
            condition = 1
    else:
        while (
            abs(robot.motor_l.angle()) < target_angle_l
        ):
            robot.motor_r.dc(-30)
            robot.motor_l.dc(30)
            condition = 0

    target_angle_r = abs(robot.motor_r.angle()) - final_angle_r
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)
    
    if condition:
        while (
            abs(robot.motor_r.angle()) < target_angle_r/2
        ):
            robot.motor_r.dc(-30)
            robot.motor_l.dc(30)
    else:
        while (
            abs(robot.motor_r.angle()) < target_angle_r/2
        ):  
            robot.motor_r.dc(30)
            robot.motor_l.dc(-30)

    robot.off_motors()
