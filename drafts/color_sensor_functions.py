#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Stop
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


def ev3_print(*args, **kwargs):
    ev3.screen.print(*args, **kwargs)
    print(*args, **kwargs)


def off_motors():
    motor_l.dc(0)
    motor_r.dc(0)


#######################


def forward_while_same_reflection(speed_r=50, speed_l=50, reflection_diff=10):
    starting_ref_r = sensorc2.reflection()
    starting_ref_l = sensorc1.reflection()

    stopped_l = False
    stopped_r = False
    while not stopped_l or not stopped_r:
        diff_ref_r = sensorc2.reflection() - starting_ref_r
        diff_ref_l = sensorc1.reflection() - starting_ref_l
        if abs(diff_ref_r) < reflection_diff:
            motorC.dc(speed_r)
        else:
            if not stopped_l:
                stopped_l = True
            motorC.hold()

        if abs(diff_ref_l) < reflection_diff:
            motorB.dc(speed_l)
        else:
            if not stopped_r:
                stopped_r = True
            motorB.hold()
    motorB.dc(0)
    motorC.dc(0)


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


def check_land_position_by_color(sensorc1: ColorSensor, sensorc2: ColorSensor) -> str:
    color_left = accurate_color(sensorc1.rgb())
    color_right = accurate_color(sensorc2.rgb())

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


def align_pid(target=30, kp=0.7, ki=0, kd=0):
    left_error_i = 0
    right_error_i = 0
    left_prev_error = 0
    right_prev_error = 0

    has_stopped_left = False
    has_stopped_right = False
    while not has_stopped_left and not has_stopped_right:
        left_error = color_l.reflection() - target
        right_error = color_r.reflection() - target

        left_error_i += left_error
        right_error_i += right_error

        left_error_d = left_error - left_prev_error
        right_error_d = right_error - right_prev_error

        left_prev_error = left_error
        right_prev_error = right_error

        ev3_print(left_error, left_error_i, left_error_d)
        left_pid_speed = kp * left_error + ki * left_error_i + kd * left_error_d
        right_pid_speed = kp * right_error + ki * right_error_i + kd * right_error_d

        # Limitante de velocidade
        left_speed_sign = -1 if left_pid_speed < 0 else 1
        left_pid_speed = min(75, abs(left_pid_speed)) * left_speed_sign

        right_speed_sign = -1 if right_pid_speed < 0 else 1
        right_pid_speed = min(75, abs(right_pid_speed)) * right_speed_sign

        motor_l.dc(left_pid_speed)
        motor_r.dc(right_pid_speed)


########################


def segue_linha_c1(vel, tempo):
    valorLum = 35  # medir na linha toda vez
    Kp = 3.5
    Ki = 0.05
    Kd = 10

    erro = 0
    valorI = 0
    t = 0
    cronometro.reset()
    while True:
        erro0 = erro
        erro = valorLum - sensorc1.reflection()
        valorP = erro * Kp
        if -3 < erro < 3:
            valorI = (valorI + erro) * Ki
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if tempoDecor < 1:
            tempoDecor = 1
        valorD = ((erro - erro0) * Kd) / tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel + valorPID)
        motorB.run(vel - valorPID)

        if cronometro.time() > tempo:
            break
    motorC.hold()
    motorB.hold()


def segue_linha_c1_buraco(vel):
    valorLum = 35  # medir na linha toda vez
    Kp = 3.5
    Ki = 0.05
    Kd = 10

    erro = 0
    valorI = 0
    t = 0

    cores = []
    cores_validas = [Color.YELLOW, Color.BLUE, Color.RED]

    cronometro.reset()
    while True:
        erro0 = erro
        erro = valorLum - sensorc1.reflection()
        valorP = erro * Kp
        if -3 < erro < 3:
            valorI = (valorI + erro) * Ki
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if tempoDecor < 1:
            tempoDecor = 1
        valorD = ((erro - erro0) * Kd) / tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel + valorPID)
        motorB.run(vel - valorPID)

        if sensorc2.color() not in cores and sensorc2.color() in cores_validas:
            cores.append(sensorc2.color())
            cores_validas.remove(sensorc2.color())

        if sensorc2.color() == None:
            break
    motorC.hold()
    motorB.hold()

    return cores


def segue_linha_c2_buraco(vel, array):
    valorLum = 35  # medir na linha toda vez
    Kp = 3.5
    Ki = 0.05
    Kd = 10

    erro = 0
    valorI = 0
    t = 0

    cores_validas = [Color.YELLOW, Color.BLUE, Color.RED]
    print(array)
    cronometro.reset()
    while True:
        erro0 = erro
        erro = valorLum - sensorc2.reflection()
        valorP = erro * Kp
        if -3 < erro < 3:
            valorI = (valorI + erro) * Ki
        t0 = t
        t = cronometro.time()
        tempoDecor = t - t0
        if tempoDecor < 1:
            tempoDecor = 1
        valorD = ((erro - erro0) * Kd) / tempoDecor

        valorPID = valorP + valorI + valorD

        motorC.run(vel - valorPID)
        motorB.run(vel + valorPID)

        if sensorc1.color() not in array and sensorc1.color() in cores_validas:
            array.append(sensorc1.color())
            break

    motorC.hold()
    motorB.hold()

    return array


####################

while True:
    ev3_print(accurate_color(color_l.rgb()))
    wait(200)
