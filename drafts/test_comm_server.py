#!/usr/bin/env pybricks-micropython
import math
import time

from pybricks.ev3devices import ColorSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.messaging import (
    BluetoothMailboxServer,
    LogicMailbox,
    NumericMailbox,
    TextMailbox,
)
from pybricks.parameters import Color, Port, Stop
from pybricks.tools import StopWatch, wait

# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize.
motorB = Motor(Port.B)
motorC = Motor(Port.C)
motor_l = motorB
motor_r = motorC
# motorA = Motor(Port.A)
cronometro = StopWatch()
sensorc1 = ColorSensor(Port.S1)
sensorc2 = ColorSensor(Port.S2)
color_l = sensorc1
color_r = sensorc2

# Before running this program, make sure the client and server EV3 bricks are
# paired using Bluetooth, but do NOT connect them. The program will take care
# of establishing the connection.
# The server must be started before the client!


def ev3_print(*args, **kwargs):
    ev3.screen.print(*args, **kwargs)
    print(*args, **kwargs)


def turn(angle, speed=50):
    motor_degrees = robot_axis_to_motor_degrees(angle)

    initial_angle_l = motor_l.angle()
    initial_angle_r = motor_r.angle()

    while abs(initial_angle_l - motor_l.angle()) < abs(motor_degrees) and abs(
        initial_angle_r - motor_r.angle()
    ) < abs(motor_degrees):
        motor_r.dc(speed)
        motor_l.dc(-speed)


def off_motors():
    motor_l.dc(0)
    motor_r.dc(0)


WHEEL_DIAMETER = 5.5
WHEEL_DIST = 15.3


def robot_axis_to_motor_degrees(axis_degrees):
    return axis_degrees * (WHEEL_DIST / WHEEL_DIAMETER)


def cm_to_motor_degrees(cm):
    return cm * (360 / (math.pi * WHEEL_DIAMETER))


def walk_cm(cm, speed=50, speed_l=None, speed_r=None):
    if speed_l is None:
        speed_l = speed
    if speed_r is None:
        speed_r = speed

    degrees = cm_to_motor_degrees(cm)

    initial_angle_l = motor_l.angle()
    initial_angle_r = motor_r.angle()

    while abs(initial_angle_l - motor_l.angle()) < abs(degrees) and abs(
        initial_angle_r - motor_r.angle()
    ) < abs(degrees):
        motor_r.dc(speed_r)
        motor_l.dc(speed_l)


def accurate_color(rgb_tuple):
    if sum(rgb_tuple) == 0:
        return None
    if rgb_tuple[0] > 40 and rgb_tuple[1] > 40 and rgb_tuple[2] > 40:
        return Color.WHITE
    if rgb_tuple[0] in range(60, 75) and rgb_tuple[1] in range(30, 45):
        return Color.YELLOW
    if rgb_tuple[0] < 15 and rgb_tuple[1] > 15 and rgb_tuple[2] < 15:
        return Color.GREEN
    if rgb_tuple[0] < 15 and rgb_tuple[1] < 15 and rgb_tuple[2] < 15:
        return Color.BLACK


def check_land_position_by_color(color_l: ColorSensor, color_r: ColorSensor) -> str:
    color_left = accurate_color(color_l.rgb())
    color_right = accurate_color(color_r.rgb())

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


def forward_while_same_reflection(speed_r=50, speed_l=50, reflection_diff=10):
    starting_ref_r = color_r.reflection()
    starting_ref_l = color_l.reflection()

    stopped_l = False
    stopped_r = False
    while not stopped_l or not stopped_r:
        diff_ref_r = color_r.reflection() - starting_ref_r
        diff_ref_l = color_l.reflection() - starting_ref_l
        if abs(diff_ref_r) < reflection_diff:
            motor_r.dc(speed_r)
        else:
            if not stopped_l:
                stopped_l = True
            motor_r.hold()

        if abs(diff_ref_l) < reflection_diff:
            motor_l.dc(speed_l)
        else:
            if not stopped_r:
                stopped_r = True
            motor_l.hold()
    motor_l.dc(0)
    motor_r.dc(0)


def katara_position_routine():
    while True:
        forward_while_same_reflection(60, 80)
        location = check_land_position_by_color(color_l, color_r)
        ev3_print(location)

        if location == "EDGE":
            walk_cm(10, -50)
            turn(90)
        elif ":" in location:
            left_loc, right_loc = location.split(":")
            if right_loc == "EDGE":
                # Lida com motor direito na borda
                ev3.light.on(Color.GREEN)
                ev3.speaker.beep()
                walk_cm(5, -50)
                turn(45 if left_loc == "RAMP" else 180)
                ev3.light.off()
            elif left_loc == "EDGE":
                ev3.light.on(Color.ORANGE)
                ev3.speaker.beep()
                # Lida com motor esquerdo na borda
                walk_cm(5, -50)
                turn(45 if left_loc == "RAMP" else 180, speed=-50)
                ev3.light.off()
            else:
                walk_cm(5, -50)
                turn(45)
        # elif "RAMP" not in location:
        #     walk_cm(10, -50)
        #     turn(180)
        else:
            break
    off_motors()
    # ROBO NA RAMPA

    walk_cm(70, speed_l=50, speed_r=55)
    turn(90, speed=-50)
    forward_while_same_reflection()
    walk_cm(10, -50)
    turn(90)
    off_motors()


server = BluetoothMailboxServer()
print("waiting for connection...")
server.wait_for_connection()
print("connected!")

ev3_print("1")
logic_mbox = LogicMailbox("start", server)
ev3_print("2")
logic_mbox.wait()
ev3_print("3")
start = logic_mbox.read()
ev3_print(start)
if start:
    katara_position_routine()
    logic_mbox.send(True)
