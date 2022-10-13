import time
import math

from pybricks.parameters import Color
from pybricks.tools import wait

import constants as const
from robot import Robot
from utils import accurate_color, ev3_print


# def duct_ends(
#     robot: Robot,
#     speed: int = 20,
#     dir_sign: int = 1,  # 1 ou -1
# ):
#     """
#     Com os dois ultrassonicos da frente apontando para um duto,
#     o robô gira até estar de frente para a ponta do tubo.
#     - `dir_sign` é 1 ou -1 e representa a direção da curva.
#     """

#     robot.motor_l.reset_angle(0)
#     robot.motor_r.reset_angle(0)

#     if dir_sign == 1:
#         sensor = robot.ultra_front_r
#         robot.ultra_front_l.distance(silent=True)
#     elif dir_sign == -1:
#         sensor = robot.ultra_front_l
#         robot.ultra_front_r.distance(silent=True)

#     previous_distance = sensor.distance()
#     while abs(previous_distance - sensor.distance()) < const.DUCT_ENDS_US_DIFF:
#         previous_distance = sensor.distance()
#         robot.motor_l.dc(dir_sign * speed)
#         robot.motor_r.dc(dir_sign * -speed)
#         ev3_print(
#             sensor.distance(),
#             ev3=robot.brick,
#         )

#     robot.off_motors()

#     return (abs(robot.motor_l.angle()) + abs(robot.motor_l.angle()))/4

def duct_ends(
    robot: Robot,
    speed: int = 25,
):
    """
    Com o ultrassonico da frente apontando para um duto,
    o robô gira até estar de frente para a ponta do tubo,
    depois faz o mesmo para o lado oposto e aponta para o centro.
    Retorna o tamanho do duto lido.
    """

    lowest_ultra_value_l = 2550
    lowest_ultra_value_r = 2550
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)
    sensor = robot.ultra_front_r

    # robot.stopwatch.reset()
    # if(sensor.distance() > const.DIST_LINE_TO_END):
    #     while True:
    #         robot.motor_l.dc(speed)
    #         robot.motor_r.dc(-speed)
    #         if(sensor.distance() < const.DIST_LINE_TO_END):
    #             robot.pid_turn(20)
    #             break
    #         elif(robot.stopwatch.time()>3000):
    #             while sensor.distance() < const.DUCT_ENDS_US_DIFF:
    #                 robot.motor_l.dc(-speed)
    #                 robot.motor_r.dc(speed)
    #                 robot.pid_turn(-20)
    #                 break
    # wait(500)

    while sensor.distance() < const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(speed)
        robot.motor_r.dc(-speed)
        lowest_ultra_value_r = min(lowest_ultra_value_r,sensor.distance())

    while sensor.distance() > const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(-speed)
        robot.motor_r.dc(speed)
    
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)
    while sensor.distance() < const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(-speed)
        robot.motor_r.dc(speed)
        lowest_ultra_value_l = min(lowest_ultra_value_l,sensor.distance())

    robot.off_motors()
    ultra_mean = (lowest_ultra_value_l+lowest_ultra_value_r)/2
    motor_mean = (abs(robot.motor_l.angle()) + abs(robot.motor_l.angle()))/2
    robot.pid_turn(motor_mean/2,mode=2)

    # full_circle = 2 * math.pi * (ultra_mean/10)
    # length = (theta * full_circle)/(2 * math.pi)
    # print("theta=",theta,"ultra=",ultra_mean,"circle=",full_circle,"length=",length)
    # return length
    x = (const.WHEEL_DIAMETER*motor_mean)/(const.WHEEL_DIST)
    print(motor_mean,x)

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
    """encontra a menor distância num arco de 90º na frente do robô"""
    lowest_ultra_value = 2550
    robot.pid_turn(-45)
    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)
    ultra_reads = []
    while True:
        distance = robot.ultra_front_r.distance()
        ultra_reads.append(distance)
        lowest_ultra_value = min(ultra_reads)
        m_mean = (abs(robot.motor_l.angle())+abs(robot.motor_r.angle()))/2
        robot.motor_l.dc(30)
        robot.motor_r.dc(-30)
        if(len(ultra_reads)>=10):
            if(ultra_reads[-1]-ultra_reads[-2]<0 and sum(ultra_reads[-10:])/10 < const.DIST_LINE_TO_END):
                duct_ends(robot)
                robot.off_motors()
                return lowest_ultra_value

        if(m_mean>robot.robot_axis_to_motor_degrees(90)):
            robot.pid_turn(-45)
            robot.off_motors()
            return False
        
    