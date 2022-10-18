import time

from pybricks.parameters import Color
from pybricks.tools import wait

import constants as const
from robot import Robot
from utils import wait_button_pressed


def gas_duct_routine(robot: Robot, delivery=None):
    robot.forward_while_same_reflection(reflection_diff=2)
    robot.pid_walk(5, -80)
    robot.pid_turn(-90)

    robot.move_to_distance(const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front)
    robot.pid_turn(-90)

    robot.min_aligner(min_function=robot.infra_side.distance)

    while True:
        wall_flw_value = robot.pid_wall_follower(front_sensor=robot.ultra_front)
        if wall_flw_value == 1:
            # Curva pra dentro ou buraco
            while robot.infra_side.distance() < const.WALL_SEEN_DIST:
                robot.motor_l.dc(30)
                robot.motor_r.dc(30)
            robot.off_motors()

            robot.simple_walk(1, 30)
            if check_hole(robot):
                # buraco
                measured_value = duct_measure_hole(robot)
                # ###
                robot.brick.speaker.beep()
                robot.ev3_print(measured_value)
                wait_button_pressed(robot.brick)
                # ###

                if delivery is not None and measured_value > 0:
                    if delivery == measured_value:
                        duct_deliver(robot, measured_value)
                        delivery = None
                else:
                    return measured_value
            else:
                # curva pra dentro
                duct_follower_turn_routine(robot)
        elif wall_flw_value == 2:
            # Curva pra fora
            robot.move_to_distance(
                const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front
            )
            robot.pid_turn(-90)
        else:
            # Chegou na borda do mapa
            break
        robot.min_aligner(min_function=robot.infra_side.distance)
    if wall_flw_value == 3:
        armagedon_the_end_of_times(robot)
    robot.off_motors()


def duct_follower_turn_routine(robot: Robot, speed=const.SEARCH_WALL_SPEED):
    robot.ev3_print(duct_follower_turn_routine.__name__)
    robot.brick.speaker.beep()
    # wait_button_pressed(robot.brick)
    robot.pid_walk(14)
    robot.pid_turn(90)

    robot.ev3_print("infra_side:", robot.infra_side.distance())
    while robot.infra_side.distance() > const.WALL_SEEN_DIST:
        robot.ev3_print("infra_side:", robot.infra_side.distance())
        robot.motor_l.dc(speed)
        robot.motor_r.dc(speed)

    robot.off_motors()
    robot.pid_walk(3)
    robot.brick.speaker.beep()
    # wait_button_pressed(robot.brick)


def check_hole(robot: Robot):
    """
    True caso seja um pedaço faltante do gasoduto,
    False caso seja um "final" de gasoduto (curva).
    """
    robot.motor_sensor.run_target(300, 550)
    hole_seen = robot.infra_side.distance() <= const.WALL_SEEN_DIST
    if hole_seen:
        robot.min_aligner(robot.infra_side.distance)
    robot.motor_sensor.run_target(300, 0)
    return hole_seen


def duct_measure_hole(robot: Robot):
    robot.ev3_print(duct_measure_hole.__name__)
    robot.walk_to_hole(mode=1)

    robot.pid_walk(cm=5, vel=-60)
    robot.walk_to_hole(mode=2)

    measurement = robot.hole_measurement()

    robot.ev3_print("MEASURE:", measurement)
    if measurement > 17:
        measured_value = 20
    elif measurement > 12:
        measured_value = 15
    elif measurement >= 10:
        measured_value = 10
    else:
        measured_value = 0
    robot.ev3_print("S_MEASURE:", measured_value)
    return measured_value


def armagedon_the_end_of_times(robot: Robot):
    robot.brick.light.on(Color.RED)
    robot.ev3_print("THIS IS THE\nARMAGEDON\nTHE END OF TIMES")
    while True:
        robot.brick.speaker.beep()
        robot.brick.speaker.beep(100)
        robot.brick.speaker.beep(300)


def duct_deliver(robot: Robot, measured_value: int):
    # Abaixa o sensor enquanto faz ré
    robot.motor_sensor.run_target(300, 600, wait=False)
    robot.simple_walk((-(measured_value + 2) / 2) - 1.5, speed=30)
    robot.motor_sensor.run_target(300, 600)

    # Alinha com o sensor do lado
    robot.min_aligner(robot.infra_side.distance)

    robot.pid_turn(90)
    robot.min_aligner(robot.ultra_front.distance, acceptable_range=30)

    # Entrega
    robot.move_to_distance(45, sensor=robot.ultra_front)

    robot.motor_claw.run_target(100, 110)

    robot.pid_walk(1, 30)
    robot.motor_claw.run_target(100, 95)

    # Afasta levantando a garra e o sensor
    robot.pid_walk(5, -80)
    robot.motor_claw.run_target(300, 300, wait=False)
    robot.motor_sensor.run_target(300, 0, wait=False)
    robot.move_to_distance(const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front)

    # Curva final (e wait da garra e sensor)
    robot.pid_turn(-90)
    robot.motor_sensor.run_target(300, 0, wait=True)
    robot.motor_claw.run_target(300, 300, wait=True)

    robot.simple_walk(((measured_value + 2) / 2) + 4, speed=30)


def duct_get(robot: Robot):
    """Recolhe o duto na terra"""
    # recolhe o duto
    robot.move_to_distance(120, robot.ultra_front)

    robot.motor_claw.run_target(300, 0)

    robot.pid_walk(cm=14, vel=30)
    robot.off_motors()
    robot.motor_claw.run_target(300, 300)
    robot.pid_turn(180)
    robot.forward_while_same_reflection()
    robot.pid_walk(cm=2, vel=-30)
    robot.pid_align()
