"""
Módulo para funções que envolvem o problema da localização/mapeamento no mapa da larc
"""

from pybricks.parameters import Color

from src.robot import Robot
from src.utils import accurate_color, ev3_print


def check_land_position_by_color(robot: Robot) -> str:
    color_left = accurate_color(robot.color_l.rgb())
    color_right = accurate_color(robot.color_r.rgb())

    if color_left == Color.GREEN:
        pos_left = "RAMP"
    elif color_left is None:
        pos_left = "EDGE"
    else:
        pos_left = str(color_left)

    if color_right == Color.GREEN:
        pos_right = "RAMP"
    elif color_right is None:
        pos_right = "EDGE"
    else:
        pos_right = str(color_right)

    if pos_left.startswith("Color") and pos_right.startswith("Color"):
        return "COLOR"
    if pos_left == pos_right:
        return pos_left

    return str(pos_left + ":" + pos_right)


def katara_position_routine(robot: Robot):
    while True:
        robot.forward_while_same_reflection(60, 80)
        location = check_land_position_by_color(robot)
        ev3_print(location, ev3=robot.brick)

        if location == "EDGE":
            robot.simple_walk(10, -50)
            robot.simple_turn(90)
        elif ":" in location:
            left_loc, right_loc = location.split(":")
            if right_loc == "EDGE":
                # Lida com motor direito na borda
                robot.brick.light.on(Color.GREEN)
                robot.brick.speaker.beep()
                robot.simple_walk(5, -50)
                robot.simple_turn(45 if left_loc == "RAMP" else 180)
                robot.brick.light.off()
            elif left_loc == "EDGE":
                robot.brick.light.on(Color.ORANGE)
                robot.brick.speaker.beep()
                # Lida com motor esquerdo na borda
                robot.simple_walk(5, -50)
                robot.simple_turn(45 if left_loc == "RAMP" else 180, speed=-50)
                robot.brick.light.off()
            else:
                robot.simple_walk(5, -50)
                robot.simple_turn(45)
        elif "RAMP" not in location:
            robot.simple_walk(10, -50)
            robot.simple_turn(180)
        else:
            break
    robot.off_motors()
    # ROBO NA RAMPA

    robot.simple_walk(70, speed_l=50, speed_r=55)
    robot.simple_turn(90, speed=-50)
    robot.forward_while_same_reflection()
    robot.simple_walk(10, -50)
    robot.simple_turn(90)
    robot.off_motors()
