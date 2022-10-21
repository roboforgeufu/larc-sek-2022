import time

from pybricks.parameters import Color
from pybricks.tools import wait

import constants as const
from robot import Robot
from utils import wait_button_pressed


def gas_duct_routine(robot: Robot, delivery=None):
    turn_counter = 1

    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)
    robot.pid_walk(10, -40)
    robot.pid_turn(-90)

    robot.move_to_distance(const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front)
    robot.pid_turn(-90)

    while True:
        if robot.infra_side.distance() < const.WALL_SEEN_DIST:
            robot.min_aligner(
                min_function=robot.infra_side.distance,
                motor_correction=const.KATARA_MIN_ALGN_CORRECTION,
            )
            wall_flw_value = robot.pid_wall_follower(front_sensor=robot.ultra_front)
        else:
            wall_flw_value = 1
        robot.ev3_print("WFLWVL:", wall_flw_value)
        if wall_flw_value == 1:
            # Curva pra dentro ou buraco
            # check_small_gap(robot)

            if check_hole(robot):
                # buraco
                robot.brick.speaker.beep()
                measured_value = duct_measure_hole(robot)
                # ###
                robot.brick.speaker.beep()
                robot.ev3_print(measured_value)
                # wait_button_pressed(robot.brick)
                # ###

                if measured_value > 0:
                    if delivery is not None:
                        if delivery == measured_value:
                            duct_deliver(robot, measured_value)
                            delivery = None
                    else:
                        return measured_value, turn_counter
                else:
                    robot.pid_walk(3, 60)
            else:
                # curva pra dentro
                turn_counter += 1
                duct_follower_turn_routine(robot)
        elif wall_flw_value == 2:
            # Curva pra fora
            turn_counter -= 1
            robot.move_to_distance(
                const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front
            )
            robot.pid_turn(-90)
        elif wall_flw_value == 3:
            # Chegou na borda do mapa
            robot.off_motors()
            armagedon_the_end_of_times(robot)
            break
        else:
            # Perto demais, afasta
            too_close_maneuver(robot)


def duct_follower_turn_routine(robot: Robot, speed=const.SEARCH_WALL_SPEED):
    robot.ev3_print(duct_follower_turn_routine.__name__)
    robot.brick.speaker.beep(150)
    robot.brick.speaker.beep(250)
    robot.brick.speaker.beep(150)
    # wait_button_pressed(robot.brick)
    robot.pid_walk(10, vel=40)
    robot.simple_turn(90, speed=40)

    robot.ev3_print("infra_side:", robot.infra_side.distance())
    while robot.infra_side.distance() > const.WALL_SEEN_DIST:
        robot.ev3_print("infra_side:", robot.infra_side.distance())
        robot.motor_l.dc(speed)
        robot.motor_r.dc(speed)

    robot.off_motors()
    robot.pid_walk(3)
    robot.brick.speaker.beep(250)
    robot.brick.speaker.beep(150)
    robot.brick.speaker.beep(250)


def check_hole(robot: Robot):
    """
    True caso seja um pedaço faltante do gasoduto,
    False caso seja um "final" de gasoduto (curva).
    """
    robot.motor_sensor.run_target(const.INFRA_SPEED, const.INFRA_DOWN)

    #######################
    # CHECK WALL
    robot.brick.light.on(Color.ORANGE)

    wall_flw_value = robot.pid_wall_follower(
        front_sensor=robot.ultra_front,
        side_dist=const.LOW_WALL_FOLLOWER_SIDE_DIST,
        max_cm=const.GAS_DUCT_TURN_CHECK,
    )

    robot.pid_walk(4, 50)
    robot.off_motors()
    robot.brick.light.off()
    robot.ev3_print("LOW_FLW:", wall_flw_value)
    # wait_button_pressed(robot.brick)
    #######################

    robot.brick.speaker.beep(250)
    hole_seen = robot.infra_side.distance() <= const.WALL_SEEN_DIST_LOWER
    robot.ev3_print("HOLE SEEN:", hole_seen)
    # wait_button_pressed(robot.brick)

    if hole_seen:
        robot.min_aligner(
            robot.infra_side.distance, motor_correction=const.KATARA_MIN_ALGN_CORRECTION
        )
        if robot.infra_side.distance() < const.WALL_FOLLOWER_SIDE_DIST:
            robot.motor_sensor.run_target(const.INFRA_SPEED, const.INFRA_UP, wait=False)
            too_close_maneuver(robot)
    robot.motor_sensor.run_target(const.INFRA_SPEED, const.INFRA_UP)
    return hole_seen


def check_small_gap(robot: Robot):
    """Confere se é um buraco pequeno no gasoduto. Retorna True caso seja."""
    initial_motor_l = robot.motor_l.angle()
    initial_motor_r = robot.motor_r.angle()
    robot.brick.light.on(Color.ORANGE)
    while robot.infra_side.distance() > const.WALL_FOLLOWER_SIDE_DIST and (
        abs(robot.motor_l.angle() - initial_motor_l)
        < robot.cm_to_motor_degrees(const.GAS_DUCT_SMALL_GAP)
        and abs(robot.motor_r.angle() - initial_motor_r)
        < robot.cm_to_motor_degrees(const.GAS_DUCT_SMALL_GAP)
    ):
        # robot.ev3_print("CK_SMLGAP:", robot.infra_side.distance())
        robot.motor_l.dc(30)
        robot.motor_r.dc(30)
    robot.off_motors()
    robot.brick.light.off()

    duct_seen = robot.infra_side.distance() <= const.WALL_SEEN_DIST
    robot.brick.speaker.beep()
    robot.brick.light.on(Color.ORANGE)
    if not duct_seen:
        robot.move_both_to_target(target_l=initial_motor_l, target_r=initial_motor_r)
    robot.brick.light.off()
    return duct_seen


def duct_measure_hole(robot: Robot):
    robot.ev3_print(duct_measure_hole.__name__)

    robot.ev3_print("dbg: 1")
    if robot.infra_side.distance() > const.WALL_SEEN_DIST:
        # se está vendo o buraco, ré até deixar de ver
        robot.walk_to_hole(mode=1)
        robot.ev3_print("dbg: 2")
        robot.pid_walk(cm=5, vel=60)

        robot.ev3_print("dbg: 3")
        if robot.infra_side.distance() < const.WALL_SEEN_DIST:
            # se não está vendo o buraco, vai pra frente até ver, ou no máximo 7 cm (?)
            robot.walk_to_hole(mode=2)
        robot.ev3_print("dbg: 4")

    measurement = robot.hole_measurement()
    robot.ev3_print("dbg: 5")

    robot.ev3_print("MEASURE:", measurement)

    diff_to_size = [
        abs(measurement - 10),
        abs(measurement - 15),
        abs(measurement - 20),
    ]
    min_diff_idx = diff_to_size.index(min(diff_to_size))

    if min_diff_idx == 0 and min(diff_to_size) > 3:
        # Leu um valor pequeno (< 10), mas muito menor que 10 (buraco a ser ignorado)
        measured_value = 0
    elif min_diff_idx == 0:
        measured_value = 10
    elif min_diff_idx == 1:
        measured_value = 15
    elif min_diff_idx == 2:
        measured_value = 20
    else:
        measured_value = 0
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
    robot.motor_sensor.run_target(const.INFRA_SPEED, const.INFRA_DOWN, wait=False)
    robot.simple_walk((-(measured_value + 2) / 2) - 1.5, speed=30)
    robot.motor_sensor.run_target(const.INFRA_SPEED, const.INFRA_DOWN)

    # Alinha com o sensor do lado
    robot.min_aligner(
        robot.infra_side.distance, motor_correction=const.KATARA_MIN_ALGN_CORRECTION
    )

    robot.pid_turn(90)
    robot.min_aligner(
        robot.ultra_front.distance,
        acceptable_range=60,
        motor_correction=const.KATARA_MIN_ALGN_CORRECTION,
    )

    # Entrega
    robot.move_to_distance(40, sensor=robot.ultra_front, safe_max_read=150)

    robot.motor_claw.run_target(100, 110)

    robot.pid_walk(1.5, 25)
    robot.motor_claw.run_target(100, 95)

    # Afasta levantando a garra e o sensor
    robot.pid_walk(5, -80)
    robot.motor_claw.run_target(300, const.CLAW_UP, wait=False)
    robot.motor_sensor.run_target(const.INFRA_SPEED, const.INFRA_UP, wait=False)
    robot.move_to_distance(const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front)

    # Curva final (e wait da garra e sensor)
    robot.pid_turn(-90)
    robot.motor_sensor.run_target(const.INFRA_SPEED, const.INFRA_UP, wait=False)
    robot.motor_claw.run_target(300, const.CLAW_UP)

    robot.simple_walk(((measured_value + 2) / 2) + 4, speed=30)


def duct_get(robot: Robot):
    """Recolhe o duto na terra"""
    # recolhe o duto
    robot.move_to_distance(120, robot.ultra_front)

    robot.motor_claw.run_target(300, const.CLAW_DOWN)

    robot.pid_walk(cm=14, vel=30)
    robot.off_motors()
    robot.motor_claw.run_target(300, const.CLAW_UP)
    robot.pid_turn(180)
    robot.forward_while_same_reflection()
    robot.pid_walk(cm=2, vel=-30)
    robot.pid_align()


def too_close_maneuver(robot: Robot):
    # Manobra pra colocar o robô a uma distância mais apropriada do gasoduto
    robot.pid_turn(90)
    robot.move_to_distance(
        distance=const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front
    )
    robot.pid_turn(-90)
    robot.ev3_print("MNV IR:", robot.infra_side.distance())
