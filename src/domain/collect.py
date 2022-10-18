import math
import time

from pybricks.parameters import Color
from pybricks.tools import wait

import constants as const
from robot import Robot
from utils import ev3_print, PIDValues

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
    sensor = robot.ultra_front

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
        lowest_ultra_value_r = min(lowest_ultra_value_r, sensor.distance())

    while sensor.distance() > const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(-speed)
        robot.motor_r.dc(speed)

    robot.motor_l.reset_angle(0)
    robot.motor_r.reset_angle(0)
    while sensor.distance() < const.DUCT_ENDS_US_DIFF:
        robot.motor_l.dc(-speed)
        robot.motor_r.dc(speed)
        lowest_ultra_value_l = min(lowest_ultra_value_l, sensor.distance())

    robot.off_motors()
    ultra_mean = (lowest_ultra_value_l + lowest_ultra_value_r) / 2
    motor_mean = (abs(robot.motor_l.angle()) + abs(robot.motor_l.angle())) / 2
    robot.pid_turn(motor_mean / 2, mode=2)

    theta = (const.WHEEL_DIAMETER * motor_mean) / (const.WHEEL_DIST)
    theta_rad = (theta * math.pi) / 180
    arc_length = theta_rad * ((ultra_mean / 10) + 8)
    return arc_length


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
        m_mean = (abs(robot.motor_l.angle()) + abs(robot.motor_r.angle())) / 2
        robot.motor_l.dc(30)
        robot.motor_r.dc(-30)
        if len(ultra_reads) >= 10:
            if (
                ultra_reads[-1] - ultra_reads[-2] < 0
                and sum(ultra_reads[-10:]) / 10 < const.DIST_LINE_TO_END
            ):
                arc_length = duct_ends(robot)
                robot.off_motors()
                return lowest_ultra_value, arc_length

        if m_mean > robot.robot_axis_to_motor_degrees(90):
            robot.pid_turn(-45)
            robot.off_motors()
            return False, 0


def duct_seek_routine_new(robot: Robot,color):

    measurements = []
    travelled_distance = 0

    while(robot.accurate_color(robot.color_l.rgb())!=color):
        robot.motor_l.reset_angle(0)
        robot.motor_r.reset_angle(0)
        
        robot.walk_to_hole(mode=3)
        motor_mean = (robot.motor_l.angle()+robot.motor_r.angle())/2
        degrees = motor_mean
        travelled_distance = travelled_distance + degrees
        print("1",travelled_distance)
        duct_length = robot.duct_measurement()

        if duct_length > 5:
            duct_middle_cm = ((travelled_distance / 360) * const.WHEEL_LENGTH) + (duct_length / 2)
            measurements.append((duct_length,duct_middle_cm))

        travelled_distance = travelled_distance + ((duct_length * 360) / const.WHEEL_LENGTH)
        print("2",travelled_distance)
        robot.off_motors()
        wait(50)

    robot.off_motors()
    travelled_distance_cm = (travelled_distance / 360) * const.WHEEL_LENGTH
    print(measurements,travelled_distance_cm)
    
    max_duct_length = max(i for i, _ in measurements)
    optimal_motor_choice = 0
    for duct_length, motor_cm in measurements:
        if max_duct_length == duct_length:
            optimal_motor_choice = motor_cm
    

    robot.pid_walk(cm=(travelled_distance_cm-optimal_motor_choice),vel=-60)
    robot.pid_turn(-90)

    ###REFATORAR, DAR UMA OLHADA

    # recolhe o duto
    dist = robot.ultra_front.distance()/10
    robot.pid_walk(cm=max(1, (dist-3)), vel=50)
    robot.pid_walk(cm=8, vel=20)
    robot.off_motors()
    robot.motor_claw.reset_angle(0)
    robot.motor_claw.run_target(300, 300)

    # alinha com a linha preta e o buraco para deixar o duto numa posição padrão
    robot.forward_while_same_reflection(speed_r=-60, speed_l=-60)
    robot.pid_walk(cm=13, vel=-60)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection()
    robot.pid_align(PIDValues(target=30, kp=0.7, ki=0.001, kd=0.3))

    # deixa o duto a 40cm do buraco
    robot.pid_walk(cm=40, vel=-60)
    robot.pid_turn(-90)
    robot.motor_claw.run_target(300, -10)

    # alinha com o buraco restaurando a posicao inicial
    robot.pid_walk(cm=5, vel=-60)
    robot.pid_turn(180)
    robot.forward_while_same_reflection()
    robot.pid_walk(cm=5, vel=-60)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection()
    robot.pid_walk(cm=5, vel=-60)
    robot.pid_turn(90)
    robot.forward_while_same_reflection()
    robot.pid_walk(cm=10, vel=-60)
    robot.one_wheel_turn(800, robot.motor_l)
    robot.pid_line_grabber(100, 2000, robot.color_l)
