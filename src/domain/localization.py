"""
Módulo para funções que envolvem o problema da localização/mapeamento no mapa da larc
"""

from pybricks.parameters import Color

import constants as const
from robot import Robot
from utils import accurate_color, ev3_print, wait_button_pressed


def check_land_position_by_color(robot: Robot) -> str:
    """
    Identifica em qual local da área da terra o robô está (borda da meeting area com os dois
    sensores pra fora), baseado na leitura dos sensores de cor.
    """
    color_left = accurate_color(robot.color_l.rgb())
    color_right = accurate_color(robot.color_r.rgb())

    robot.ev3_print(color_left, color_right)

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
            60, 80, avoid_obstacles=True, reflection_diff=20
        )

        location = check_land_position_by_color(robot)

        # wait_button_pressed(robot.brick)

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
    # ROBO NA RAMPA

    back_to_water_routine(robot)


def land_position_routine(robot: Robot):
    """Rotina de identificação de posição no mapa do robô da terra."""

    color_order = []  # type: ignore
    while True:
        robot.forward_while_same_reflection(80, 80, 10)
        robot.pid_walk(cm=2, vel=-60)
        robot.pid_align()
        robot.pid_walk(cm=3, vel=40)
        location = check_land_position_by_color(robot)

        if location == "RAMP":

            robot.pid_accelerated_walk(-500, 3)
            robot.pid_turn(180)

        elif location == "COLOR":
            
            robot.pid_walk(cm=8, vel=-60)
            robot.one_wheel_turn(800, robot.motor_r)

            # Curva com um sensor até a linha
            # Curva com um sensor até depois da linha

            robot.pid_line_grabber(100, 2000, robot.color_r)
            color_order = robot.pid_line_follower_color_id(80, robot.color_r)
            if len(color_order) < 2:
                robot.pid_accelerated_walk(-500, 2)
                robot.pid_turn(170)
                robot.pid_line_grabber(100, 2000, robot.color_l)
                color_order = robot.pid_line_follower_color_id(
                    80, robot.color_l, color_order
                )
                ev3_print(color_order, ev3=robot.brick)
                robot.pid_accelerated_walk(-500, 2)
                robot.pid_turn(190)
                robot.pid_line_grabber(100, 2000, robot.color_r)
                robot.pid_line_follower_color_id(80, robot.color_r)
            break

        elif location == "EDGE":
            robot.pid_accelerated_walk(-500, 3)
            robot.pid_turn(-90)

        else:
            robot.pid_accelerated_walk(-500, 3)

    return color_order


def back_from_water_routine(robot: Robot):
    robot.pid_turn(-90)
    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)
    robot.pid_walk(10, -80)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)

    robot.pid_walk(10, -80)
    robot.pid_turn(90)
    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)
    robot.pid_walk(10, -80)
    robot.pid_turn(-90)
    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)

    robot.simple_walk(-40, speed=30)
    robot.pid_turn(90)
    robot.forward_while_same_reflection(reflection_diff=const.COL_REFLECTION_HOLE_DIFF)

    robot.pid_walk(30, vel=80)
    robot.forward_while_same_reflection(
        speed_r=80, speed_l=80, reflection_diff=const.COL_REFLECTION_HOLE_DIFF
    )
    robot.pid_walk(20)


def back_to_water_routine(robot: Robot):
    """O robô está na rampa, desce para a água e vira para a direita."""
    robot.simple_walk(70, speed_l=50, speed_r=55)
    robot.simple_turn(90, speed=-50)
