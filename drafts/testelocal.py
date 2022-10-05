#!/usr/bin/env pybricks-micropython
from re import I
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import StopWatch, wait
# Initialize the EV3 brick.
ev3 = EV3Brick()
# Initialize.
motor_l = Motor(Port.B)
motor_r = Motor(Port.C)
# motorA = Motor(Port.A)
stopwatch = StopWatch()
color_l = ColorSensor(Port.S1)
color_r = ColorSensor(Port.S2)

WHEEL_DIAMETER = 5.5
WHEEL_DIST = 15.3
ROTATION = ((WHEEL_DIST)/WHEEL_DIAMETER)*360


def off_motors():
    motor_l.dc(0)
    motor_r.dc(0)

def align_pid(target=30, kp=0.7, ki=0, kd=0):
    i=0
    fc = 0.9
    left_error_i = 0
    right_error_i = 0
    left_prev_error = 0
    right_prev_error = 0

    has_stopped_left = False
    has_stopped_right = False
    while not has_stopped_left and not has_stopped_right:
        left_error = color_l.reflection() - target
        right_error = color_r.reflection() - (target*fc)

        left_error_i += left_error
        right_error_i += right_error

        left_error_d = left_error - left_prev_error
        right_error_d = right_error - right_prev_error

        left_prev_error = left_error
        right_prev_error = right_error

        #ev3_print(left_error, left_error_i, left_error_d)
        left_pid_speed = kp * left_error + ki * left_error_i + kd * left_error_d
        right_pid_speed = kp * right_error + ki * right_error_i + kd * right_error_d

        # Limitante de velocidade
        left_speed_sign = -1 if left_pid_speed < 0 else 1
        left_pid_speed = min(75, abs(left_pid_speed)) * left_speed_sign

        right_speed_sign = -1 if right_pid_speed < 0 else 1
        right_pid_speed = min(75, abs(right_pid_speed)) * right_speed_sign

        motor_l.dc(left_pid_speed)
        motor_r.dc(right_pid_speed)

        if(abs(left_error) < 5 and abs(right_error) < 5):
            i=i+1
            if(i>=50):
                break

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


def line_grabber(vel,time,sensor):
    target = 35 #medir na linha toda vez
    kp = 3.5
    ki = 0.05 
    kd = 10

    error = 0
    i_share = 0
    elapsed_time = 0

    stopwatch.reset()
    while True:
        prev_error = error  
        error = target - sensor.reflection()  
        p_share = error*kp

        if(abs(error)<3): i_share = (i_share+error)*ki

        prev_elapsed_time = elapsed_time
        wait(1)
        elapsed_time = stopwatch.time()
        d_share = ((error - prev_error)*kd)/(elapsed_time-prev_elapsed_time)

        pid_correction = p_share + i_share + d_share

        pid_sign = 1 if sensor==color_l else -1
        motor_r.run(vel+(pid_correction*pid_sign))
        motor_l.run(vel-(pid_correction*pid_sign))

        if(stopwatch.time()>time): break
    off_motors()

def line_follower_color_id(vel,sensor_follow,sensor_color,array):
    target = 35 #medir na linha toda vez
    kp = 0.25
    ki = 0.003
    kd = 0.4

    error = 0
    i_share = 0
    elapsed_time = 0

    valid_colors = [Color.YELLOW,Color.BLUE,Color.RED]

    stopwatch.reset()
    while True:
        prev_error = error  
        error = target - sensor_follow.reflection()  
        p_share = error*kp

        if(abs(error)<3): i_share = (i_share+error)*ki

        prev_elapsed_time = elapsed_time
        wait(1)
        elapsed_time = stopwatch.time()
        d_share = ((error - prev_error)*kd)/(elapsed_time - prev_elapsed_time)

        pid_correction = p_share + i_share + d_share

        pid_sign = 1 if sensor_follow==color_l else -1

        motor_r.dc(vel+pid_correction*pid_sign)
        motor_l.dc(vel-pid_correction*pid_sign)

        if(sensor_color.color() not in array and sensor_color.color() in valid_colors):
            array.append(sensor_color.color())
            valid_colors.remove(sensor_color.color())

        if(sensor_color.color()==None): break

    motor_r.hold()
    motor_l.hold()

    return array

def turn_one_wheel(time,motor):

    stopwatch.reset()
    while(stopwatch.time()<time):
        vel = -(stopwatch.time()*10/time)**2+stopwatch.time()*(200/time)+20

        motor.dc(vel)
        if(motor==motor_l): motor_r.hold()
        else: motor_l.hold()

    off_motors()

def dc_accelerated(time,mode): #mode 1 termina vel 0 mode 2 vel max mode 3 começa vel max termina vel 0

    kp = 3 
    ki = 0.02
    kd = 3

    t = 0
    i_share = 0
    error = 0

    stopwatch.reset()
    motor_l.reset_angle(0)
    motor_r.reset_angle(0)

    while(stopwatch.time()<abs(time)):

        prev_error = error
        error = motor_r.angle() - motor_l.angle()
        p_share = error*kp 

        if(abs(error)<3): i_share = i_share+(error*ki)

        t0 = t
        wait(1)
        t = stopwatch.time()
        d_share = ((error - prev_error)*kd)/(t - t0)

        pid_correction = p_share+i_share+d_share

        if(mode==1):
            a=1
            b=1
            c=0
        elif(mode==2):
            a=0.5
            b=0.5
            c=0
        elif(mode==3):
            a=0.5
            b=0
            c=80

        vel = -(t*a*20/abs(time))**2+t*(b*400/abs(time))+(c+20)
        print(vel)
        sign = -1 if time<0 else 1
        motor_l.dc(sign*vel+pid_correction)
        motor_r.dc(sign*vel-pid_correction)

    motor_l.hold()
    motor_r.hold()


def turn(angle): #angle positivo: direita, negativo: esquerda
    motor_l.reset_angle(0)
    motor_r.reset_angle(0)
    kp = 3
    ki = 0.2
    kd = 8
    
    t = 0
    integ = 0
    error = 0

    ACCEPTABLE_ANGLE = 0.9*angle
    while(motor_l.angle()<= ACCEPTABLE_ANGLE):
        motor_angle_average = (motor_l.angle() - motor_r.angle())/2
        prev_error = error
        error = angle - motor_angle_average

        prop = error*kp
        if(abs(error)<3): integ = integ+(error*ki)
        t0 = t
        t = stopwatch.time()
        tempoDecor = t - t0
        if(tempoDecor<1): tempoDecor = 1
        deriv = ((error - prev_error)*kd)/tempoDecor

        correction = prop+integ+deriv
        vel = 20 + correction
        if(vel<0):
            if(vel>-20): vel = -20
        else:
            if(vel<20): vel = 20
        motor_r.run(-vel)
        motor_l.run(vel)

    motor_r.hold()
    motor_l.hold()



def toph_position_routine():
    color_order = []
    forward_while_same_reflection(80,100,10)
    location = check_land_position_by_color(color_l,color_r)

    while True:
        if(location=="RAMP"):
            align_pid(target=30, kp=0.7, ki=0.001, kd=0.3)
            dc_accelerated(-500,3)
            turn(ROTATION/2)
            forward_while_same_reflection(80,80,10)
            location = check_land_position_by_color(color_l,color_r)

        elif(location=="COLOR"):
            turn_one_wheel(800,1)
            line_grabber(100,1500,color_l)
            color_order = line_follower_color_id(100,color_l,color_r,color_order)
            if(len(color_order)<2):
                dc_accelerated(-500,2)
                turn(ROTATION/2)
                line_grabber(100,1500,color_r)
                color_order = line_follower_color_id(100,color_r,color_l,color_order)
                print(color_order)
                dc_accelerated(-500,2)
                turn(ROTATION/2)
                line_grabber(100,1500,color_l)
                line_follower_color_id(100,color_l,color_r,color_order)
            break

        elif(location=="EDGE"):
            align_pid(target=30, kp=0.7, ki=0.001, kd=0.3)
            dc_accelerated(-500,3)
            turn((-ROTATION/4)-15)
            forward_while_same_reflection(80,80,10)
            location = check_land_position_by_color(color_l,color_r)
        
        else:
            dc_accelerated(-500,3)
            forward_while_same_reflection(100,80,10)
            location = check_land_position_by_color(color_l,color_r)


toph_position_routine()
off_motors()
