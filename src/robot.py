"""
Módulo central pra controle do Robô.

Devem estar nesse módulo:
    - Classe 'Robot', com métodos e atributos para controle geral no robô
    - Estruturas de dados auxiliares aplicáveis a "qualquer" tipo de robô

Não devem estar nesse módulo:
    - Código específico de algum problema/desafio
"""

import math

from pybricks.ev3devices import ColorSensor, InfraredSensor, Motor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port


class Robot:
    """
    Classe que representa um robô genérico.
    """

    def __init__(
        self,
        wheel_diameter: float,
        wheel_distance: float,
        motor_r: Port,
        motor_l: Port,
        motor_claw: Port = None,
        infra_side: Port = None,
        infra_front: Port = None,
        infra_front_l: Port = None,
        infra_front_r: Port = None,
        ultra_side: Port = None,
        ultra_front: Port = None,
        ultra_front_l: Port = None,
        ultra_front_r: Port = None,
        color_r: Port = None,
        color_l: Port = None,
    ) -> None:
        # Brick EV3
        self.brick = EV3Brick()

        # Medidas do robô
        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance

        # Motores
        self.motor_r = Motor(motor_r)
        self.motor_l = Motor(motor_l)
        if motor_claw is not None:
            self.motor_claw = Motor(motor_claw)

        # Sensores infra vermelhos
        if infra_side is not None:
            self.infra_side = InfraredSensor(infra_side)
        if infra_front_l is not None:
            self.infra_front_l = InfraredSensor(infra_front_l)
        if infra_front_r is not None:
            self.infra_front_r = InfraredSensor(infra_front_r)
        if infra_front is not None:
            self.infra_front = InfraredSensor(infra_front)

        # Sensores ultrassonicos
        if ultra_side is not None:
            self.ultra_side = ultra_side
        if ultra_front_l is not None:
            self.ultra_front_l = UltrasonicSensor(ultra_front_l)
        if ultra_front_r is not None:
            self.ultra_front_r = UltrasonicSensor(ultra_front_r)
        if ultra_front is not None:
            self.ultra_front = UltrasonicSensor(ultra_front)

        # Sensores de cor
        if color_l is not None:
            self.color_l = ColorSensor(color_l)
        if color_r is not None:
            self.color_r = ColorSensor(color_r)

    def robot_axis_to_motor_degrees(self, axis_degrees: float):
        return axis_degrees * (self.wheel_distance / self.wheel_diameter)

    def cm_to_motor_degrees(self, cm: float):
        return cm * (360 / (math.pi * self.wheel_diameter))

    def off_motors(self):
        self.motor_l.dc(0)
        self.motor_r.dc(0)

    def forward_while_same_reflection(self, speed_r=50, speed_l=50, reflection_diff=10):
        starting_ref_r = self.color_r.reflection()
        starting_ref_l = self.color_l.reflection()

        stopped_l = False
        stopped_r = False
        while not stopped_l or not stopped_r:
            diff_ref_r = self.color_r.reflection() - starting_ref_r
            diff_ref_l = self.color_l.reflection() - starting_ref_l
            if abs(diff_ref_r) < reflection_diff:
                self.motor_r.dc(speed_r)
            else:
                if not stopped_l:
                    stopped_l = True
                self.motor_r.hold()

            if abs(diff_ref_l) < reflection_diff:
                self.motor_l.dc(speed_l)
            else:
                if not stopped_r:
                    stopped_r = True
                self.motor_l.hold()

    def simple_turn(self, angle, speed=50):
        motor_degrees = self.robot_axis_to_motor_degrees(angle)

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        while abs(initial_angle_l - self.motor_l.angle()) < abs(motor_degrees) and abs(
            initial_angle_r - self.motor_r.angle()
        ) < abs(motor_degrees):
            self.motor_r.dc(speed)
            self.motor_l.dc(-speed)

    def simple_walk(self, cm, speed=50, speed_l=None, speed_r=None):
        if speed_l is None:
            speed_l = speed
        if speed_r is None:
            speed_r = speed

        degrees = self.cm_to_motor_degrees(cm)

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        while abs(initial_angle_l - self.motor_l.angle()) < abs(degrees) and abs(
            initial_angle_r - self.motor_r.angle()
        ) < abs(degrees):
            self.motor_r.dc(speed_r)
            self.motor_l.dc(speed_l)

    def align_pid(  # pylint: disable=invalid-name
        self, target=30, kp=0.7, ki=0.001, kd=0.3
    ):
        """
        Alinha usando os dois pares motor - sensor de cor e controle PID.
        """
        i = 0
        correction_factor = 0.9
        left_error_i = 0
        right_error_i = 0
        left_prev_error = 0
        right_prev_error = 0

        has_stopped_left = False
        has_stopped_right = False
        while not has_stopped_left and not has_stopped_right:
            left_error = self.color_l.reflection() - target
            right_error = self.color_r.reflection() - (target * correction_factor)

            left_error_i += left_error
            right_error_i += right_error

            left_error_d = left_error - left_prev_error
            right_error_d = right_error - right_prev_error

            left_prev_error = left_error
            right_prev_error = right_error

            # ev3_print(left_error, left_error_i, left_error_d)
            left_pid_speed = kp * left_error + ki * left_error_i + kd * left_error_d
            right_pid_speed = kp * right_error + ki * right_error_i + kd * right_error_d

            # Limitante de velocidade
            left_speed_sign = -1 if left_pid_speed < 0 else 1
            left_pid_speed = min(75, abs(left_pid_speed)) * left_speed_sign

            right_speed_sign = -1 if right_pid_speed < 0 else 1
            right_pid_speed = min(75, abs(right_pid_speed)) * right_speed_sign

            self.motor_l.dc(left_pid_speed)
            self.motor_r.dc(right_pid_speed)

            if abs(left_error) < 5 and abs(right_error) < 5:
                i = i + 1
                if i >= 50:
                    break

    def line_grabber(self, vel, time, sensor):
        """
        O robô usa um dos sensores de cor para
        """
        target = 35  # medir na linha toda vez
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
            p_share = error * kp

            if abs(error) < 3:
                i_share = (i_share + error) * ki

            prev_elapsed_time = elapsed_time
            wait(1)
            elapsed_time = stopwatch.time()
            d_share = ((error - prev_error) * kd) / (elapsed_time - prev_elapsed_time)

            pid_correction = p_share + i_share + d_share

            pid_sign = 1 if sensor == color_l else -1
            motor_r.run(vel + (pid_correction * pid_sign))
            motor_l.run(vel - (pid_correction * pid_sign))

            if stopwatch.time() > time:
                break
        off_motors()
