"""
Módulo para funções que envolvem o problema da localização/mapeamento no mapa da larc
"""

from pybricks.parameters import Color

import constants as const
from robot import Robot
from utils import PIDValues, ev3_print, wait_button_pressed


def check_land_position_by_color(robot: Robot) -> str:
    """
    Identifica em qual local da área da terra o robô está (borda da meeting area com os dois
    sensores pra fora), baseado na leitura dos sensores de cor.
    """
    color_left = robot.accurate_color(robot.color_l.rgb())
    color_right = robot.accurate_color(robot.color_r.rgb())

    # robot.ev3_print(color_left, color_right)

    if color_left == Color.GREEN:
        pos_left = "RAMP"
    elif color_left == "None":
        pos_left = "EDGE"
    else:
        pos_left = str(color_left)

    if color_right == Color.GREEN:
        pos_right = "RAMP"
    elif color_right == "None":
        pos_right = "EDGE"
    else:
        pos_right = str(color_right)

    if pos_left.startswith("Color") and pos_right.startswith("Color"):
        return "COLOR"
    if pos_left == pos_right:
        return pos_left

    return str(pos_left + ":" + pos_right)


def water_position_routine(robot: Robot):
    """Rotina de identificação de posição no mapa do robô da água."""
    while True:
        robot.forward_while_same_reflection(
            60, 60, avoid_obstacles=True, reflection_diff=20
        )
        robot.pid_walk(2, -30)
        robot.pid_align()
        robot.pid_walk(1, 30)

        location = check_land_position_by_color(robot)

        robot.ev3_print(location)

        if location == "EDGE":
            robot.simple_walk(-10, 50)
            robot.simple_turn(90)
        elif ":" in location:
            left_loc, right_loc = location.split(":")
            if right_loc == "EDGE":
                # Lida com motor direito na borda
                robot.brick.light.on(Color.GREEN)
                robot.brick.speaker.beep()
                robot.simple_walk(-5, 50)
                robot.simple_turn(45 if left_loc == "RAMP" else 180)
                robot.brick.light.off()
            elif left_loc == "EDGE":
                robot.brick.light.on(Color.ORANGE)
                robot.brick.speaker.beep()
                # Lida com motor esquerdo na borda
                robot.simple_walk(-5, 50)
                robot.simple_turn(45 if left_loc == "RAMP" else 180, speed=-50)
                robot.brick.light.off()
            else:
                robot.simple_walk(-5, 50)
                robot.simple_turn(45)
        elif "RAMP" not in location:
            robot.simple_walk(-10, 50)
            robot.simple_turn(180)
        else:
            break
    robot.off_motors()


def land_position_routine(robot: Robot):
    """Rotina de identificação de posição no mapa do robô da terra."""

    color_order = []  # type: ignore
    while True:
        robot.forward_while_same_reflection(50, 50, 20)
        robot.pid_walk(cm=2, vel=-60)
        robot.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))
        robot.pid_walk(cm=1, vel=40)
        location = check_land_position_by_color(robot)

        robot.ev3_print(location)

        if location == "RAMP":

            robot.pid_accelerated_walk(-500, 2)
            robot.pid_turn(180)

        elif location == "COLOR":

            robot.pid_walk(cm=7, vel=-60)
            robot.certify_line_alignment_routine(
                target_color=Color.BLACK,
                sensor_color=robot.color_r,
                motor=robot.motor_r,
            )
            robot.line_follower_color_id(robot.color_r)

            robot.pid_accelerated_walk(-500, 2)
            robot.pid_turn(165)
            robot.turn_till_color(
                direction="right", sensor_color=robot.color_l, target_color=Color.BLACK
            )
            robot.line_grabber(vel=20, time=3000, sensor=robot.color_l)
            color_order = robot.line_follower_color_id(robot.color_l, array=color_order)

            robot.pid_accelerated_walk(-500, 2)
            robot.pid_turn(-165)
            robot.turn_till_color(
                direction="left", sensor_color=robot.color_r, target_color=Color.BLACK
            )
            robot.line_grabber(vel=20, time=3000, sensor=robot.color_r)
            robot.line_follower_color_id(robot.color_r)

            break

        elif location == "EDGE":
            robot.pid_accelerated_walk(-500, 2)
            robot.pid_turn(-90)

        else:
            robot.pid_accelerated_walk(-500, 2)

    return color_order


def back_from_water_routine(robot: Robot, turn_counter: int):
    robot.pid_turn(-(90 * turn_counter))
    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)
    robot.pid_walk(1, -80)
    robot.pid_turn(-90)
    ramp_follower(robot)
    robot.pid_walk(10, -80)

    robot.one_wheel_turn(robot.motor_r, -90, 80)
    robot.pid_walk(3, -80)

    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)

    robot.pid_walk(30, vel=80)
    robot.forward_while_same_reflection(
        speed_r=80, speed_l=80, reflection_diff=const.COL_REFLECTION_HOLE_DIFF
    )
    robot.pid_walk(23)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection()
    robot.pid_align()


def back_to_water_routine1(robot: Robot):
    """O robô está na rampa, desce para a água e vira para a direita."""
    robot.pid_walk(8, vel=-30)
    robot.pid_turn(180)
    robot.simple_walk(-60, speed_l=30, speed_r=30)


def back_to_water_routine2(robot: Robot):
    """2"""
    robot.simple_turn(-90)
    ramp_follower(robot)
    robot.pid_walk(12, -40)

    robot.one_wheel_turn(robot.motor_r, -90, 40)
    robot.pid_walk(3, -40)

    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)
    robot.pid_walk(12, vel=-30)

    robot.pid_turn(-90)


def ramp_follower(robot: Robot):
    while (
        robot.accurate_color(robot.color_l.rgb()) != "None"
        and robot.accurate_color(robot.color_r.rgb()) != "None"
    ):
        # robot.ev3_print(robot.accurate_color(robot.color_r.rgb()))
        if robot.accurate_color(robot.color_r.rgb()) == "None":
            robot.motor_r.dc(0)
        elif robot.accurate_color(robot.color_r.rgb()) == Color.GREEN:
            robot.motor_r.dc(80)
            robot.motor_l.dc(30)
        else:
            robot.motor_r.dc(30)
            robot.motor_l.dc(80)

        if robot.accurate_color(robot.color_l.rgb()) == "None":
            robot.motor_l.dc(0)
    robot.off_motors()
