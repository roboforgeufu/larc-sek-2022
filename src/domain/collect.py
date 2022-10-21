import math
import time

from pybricks.parameters import Color
from pybricks.tools import wait

import constants as const
from robot import Robot
from utils import PIDValues, ev3_print

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
):
    """
    Com o ultrassonico da frente apontando para um duto,
    o robô gira até estar de frente para a ponta do tubo,
    depois faz o mesmo para o lado oposto e aponta para o centro.
    Retorna o tamanho do duto lido.
    """

    robot.reset_both_motor_angles()

    robot.move_until_beginning_of_duct()
    robot.move_until_beginning_of_duct(inverted=True)
    #####

    robot.move_until_end_of_duct()

    robot.reset_both_motor_angles()
    robot.move_until_beginning_of_duct(inverted=True)
    robot.move_until_end_of_duct(inverted=True)

    limiter = 1.1
    motor_correction = (
        (abs(robot.motor_l.angle()) + abs(robot.motor_l.angle())) / 4
    ) * limiter
    robot.move_both_to_target(target_l=-motor_correction, target_r=motor_correction)
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


#########################


def duct_seek_routine_new(robot: Robot, color):

    measurements = []
    travelled_distance = 0

    while robot.accurate_color(robot.color_l.rgb()) != color:
        robot.motor_l.reset_angle(0)
        robot.motor_r.reset_angle(0)

        robot.brick.light.on(Color.ORANGE)
        robot.walk_till_duct(color_check_color=color, color_check_sensor=robot.color_l)

        robot.brick.light.off()
        robot.brick.speaker.beep()

        motor_mean = robot.get_motor_mean()
        degrees = motor_mean
        travelled_distance = travelled_distance + degrees
        robot.brick.light.on(Color.RED)

        duct_length = robot.duct_measurement_new(
            color_check_color=color, color_check_sensor=robot.color_l
        )

        robot.brick.light.off()
        robot.brick.speaker.beep()

        if duct_length > 5:
            duct_middle_cm = ((travelled_distance / 360) * const.WHEEL_LENGTH) + (
                duct_length / 2
            )
            measurements.append((duct_length, duct_middle_cm))

        travelled_distance = travelled_distance + (
            (duct_length * 360) / const.WHEEL_LENGTH
        )
        robot.off_motors()

    robot.off_motors()

    travelled_distance_cm = (travelled_distance / 360) * const.WHEEL_LENGTH
    print(measurements, travelled_distance_cm)

    max_duct_length = max(i for i, _ in measurements)
    optimal_motor_choice = 0
    found_idx = 0
    for e, (duct_length, motor_cm) in enumerate(measurements):
        if max_duct_length == duct_length:
            optimal_motor_choice = motor_cm
            found_idx = e
    print(optimal_motor_choice, measurements[found_idx])

    line_grabber_distance_cm = 0
    if color != "None":
        line_grabber_distance = robot.line_grabber(
            vel=20, time=3000, sensor=robot.color_l
        )
        line_grabber_distance_cm = (line_grabber_distance / 360) * const.WHEEL_LENGTH

    else:
        robot.pid_walk(cm=3, vel=-70)
        robot.forward_while_same_reflection()

    distance_correction = 1.5
    distance_result = (
        travelled_distance_cm
        + line_grabber_distance_cm
        + distance_correction
        - optimal_motor_choice
    )
    print(line_grabber_distance_cm, distance_result)

    robot.pid_walk(cm=(distance_result), vel=-40)
    robot.pid_turn(-90)
    robot.pid_walk(cm=5, vel=-50)
    robot.forward_while_same_reflection()
    robot.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))

    ###REFATORAR, DAR UMA OLHADA
    dist = robot.ultra_front.distance()
    dist = max(1, dist - 5)
    robot.move_to_distance(50, sensor=robot.ultra_front, max_cm=35)
    # robot.duct_ends() ###################################################################
    robot.pid_walk(cm=8, vel=20)
    robot.off_motors()
    robot.motor_claw.reset_angle(0)
    robot.motor_claw.run_target(300, const.CLAW_UP)

    robot.black_line_alignment_routine()
    robot.leaves_duct_at_correct_place()


def return_to_idle_position(robot: Robot):
    # alinha com o buraco restaurando a posicao inicial
    robot.pid_walk(cm=5, vel=-60)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection()
    robot.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))
    robot.pid_walk(cm=5, vel=-60)
    robot.pid_turn(90)
    robot.forward_while_same_reflection()
    robot.pid_walk(cm=5, vel=-60)
    robot.forward_while_same_reflection()
    robot.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))
