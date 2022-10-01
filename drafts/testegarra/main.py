#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
motorA = Motor(Port.A)
cronometro = StopWatch()

# Write your program here.

motorA.run_until_stalled(-500, then=Stop.HOLD, duty_limit=40)


i=0
vel=10

cronometro.reset()
motorA.reset_angle(0)

# while(motorA.angle()<340):
#     motorA.dc(100)

while(motorA.angle()<100):
    vel=vel+0.3
    i=0
    if(vel>100): vel=100
    motorA.dc(vel)
print(vel)

while(motorA.angle()>100 and motorA.angle()<250):
    vel=vel+1
    i=0
    if(vel>100): vel=100
    motorA.dc(vel)
print(vel)

while(motorA.angle()<340 and cronometro.time()<750):
    vel=vel-1
    if(vel<20): vel=20
    motorA.dc(vel)
print(vel)

print(motorA.angle())
print(cronometro.time())
motorA.brake()
