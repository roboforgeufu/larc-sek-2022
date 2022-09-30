#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import InfraredSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import StopWatch

ev3 = EV3Brick()

motor_l = Motor(Port.B)
motor_r = Motor(Port.C)
motor_claw = Motor(Port.A)
cronometro = StopWatch()
infra_side = InfraredSensor(Port.S3)
infra_front = InfraredSensor(Port.S2)


target = 10
state = 1
vel = 100
while True:
    print(state)
    motor_r.run(vel)
    if state == 1:
        motor_l.run(vel)
        if infra_side.distance() < target:
            state = 2
        elif infra_side.distance() > target:
            state = 3
    elif state == 2:
        motor_l.run(vel - 40)
        if infra_side.distance() >= target:
            state = 1
    elif state == 3:
        motor_l.run(vel + 40)
        if infra_side.distance() >= target:
            state = 1
