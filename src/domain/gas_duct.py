import time

from pybricks.parameters import Color

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
            robot.pid_walk(1, vel=30)
            if check_hole(robot):
                # buraco
                duct_measure_hole(robot)
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
    robot.motor_sensor.run_target(300, 600)
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

    robot.ev3_print(measurement)
    if measurement > 17:
        measured_value = 20
    elif measurement > 12:
        measured_value = 15
    else:
        measured_value = 10
    robot.ev3_print(measured_value, "cm")
    return measured_value


def armagedon_the_end_of_times(robot: Robot):
    robot.brick.light.on(Color.RED)
    robot.ev3_print("THIS IS THE\nARMAGEDON\nTHE END OF TIMES")
    while True:
        robot.brick.speaker.beep()
        robot.brick.speaker.beep(100)
        robot.brick.speaker.beep(300)
