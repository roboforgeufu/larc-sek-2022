#!/usr/bin/env pybricks-micropython

# Before running this program, make sure the client and server EV3 bricks are
# paired using Bluetooth, but do NOT connect them. The program will take care
# of establishing the connection.

# The server must be started before the client!

from pybricks.messaging import BluetoothMailboxClient, TextMailbox, NumericMailbox

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

ev3 = EV3Brick()
motorB = Motor(Port.B)

# This is the name of the remote EV3 or PC we are connecting to.
SERVER = 'ev3-Server'

client = BluetoothMailboxClient()
#mbox = TextMailbox('greeting', client)
mboxteste = NumericMailbox('velocidade', client)

print('establishing connection...')
client.connect(SERVER)
print('connected!')

# In this program, the client sends the first message and then waits for the
# server to reply.

#mbox.send('hello!')
#print(mbox.read())

mboxteste.send(1)
mboxteste.wait()
vel = mboxteste.read()
print(mboxteste.read())
while True:
    motorB.run(vel)