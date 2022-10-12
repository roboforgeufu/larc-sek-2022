import time

from pybricks.parameters import Color
from pybricks.tools import wait

import constants as const
from robot import Robot
from utils import accurate_color, ev3_print


def duct_ends(
    robot: Robot,
    speed: int = 25,
):
    """
    Com os dois ultrassonicos da frente apontando para um duto,
    o robô gira até estar de frente para a ponta do tubo.
    """
    robot.ultra_front_l.distance()
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)

    sensor = robot.ultra_front_l

    robot.stopwatch.reset()
    if(sensor.distance() > 420):
        while True:
            robot.motor_l.dc(speed)
            robot.motor_r.dc(-speed)
            if(sensor.distance() < 420):
                robot.pid_turn(20)
                break
            elif(robot.stopwatch.time()>3000):
                while sensor.distance() < const.DUCT_ENDS_US_DIFF:
                    robot.motor_l.dc(-speed)
                    robot.motor_r.dc(speed)
                    robot.pid_turn(-20)
                    break
    wait(500)
    while sensor.distance() < const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(speed)
        robot.motor_r.dc(-speed)
        # ev3_print(
        #     sensor.distance(),
        #     ev3=robot.brick,
        # )
    while sensor.distance() > const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(-speed)
        robot.motor_r.dc(speed)
    
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)
    while sensor.distance() < const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(-speed)
        robot.motor_r.dc(speed)

    robot.off_motors()
    return (abs(robot.motor_l.angle()) + abs(robot.motor_l.angle()))/4


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
    robot.pid_turn(-45)
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)
    while True:
        distance = robot.ultra_front_l.distance()
        m_mean = (abs(robot.motor_l.angle())+abs(robot.motor_r.angle()))/2
        robot.motor_l.dc(30)
        robot.motor_r.dc(-30)

        if(distance<420):
            return distance

        if(m_mean>robot.robot_axis_to_motor_degrees(90)):
            robot.pid_turn(-45)
            return False
    