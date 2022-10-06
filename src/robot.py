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
from pybricks.parameters import Color, Port
from pybricks.tools import StopWatch, wait

from src import constants as const
from src.utils import PIDValues


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

        # Cronometro
        self.stopwatch = StopWatch()

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
        """Grau relativo ao eixo do robô -> grau nas rodas (motores) do robô"""
        return axis_degrees * (self.wheel_distance / self.wheel_diameter)

    def cm_to_motor_degrees(self, cm: float):
        """Distância em centímetros -> grau nas rodas (motores) do robô"""
        return cm * (360 / (math.pi * self.wheel_diameter))

    def off_motors(self):
        """Desliga motores de locomoção."""
        self.motor_l.dc(0)
        self.motor_r.dc(0)

    def forward_while_same_reflection(
        self,
        speed_r=50,
        speed_l=50,
        reflection_diff=10,
    ):
        """
        Move ambos os motores (de forma individual) até que a intensidade de reflexão
        mude o suficiente (`reflection_diff`)
        """
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
        """Curva simples"""
        motor_degrees = self.robot_axis_to_motor_degrees(angle)

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        while abs(initial_angle_l - self.motor_l.angle()) < abs(motor_degrees) and abs(
            initial_angle_r - self.motor_r.angle()
        ) < abs(motor_degrees):
            self.motor_r.dc(speed)
            self.motor_l.dc(-speed)

    def one_wheel_turn(self, time, motor: Motor):
        """
        Curva temporizada com uma roda.
        """
        self.stopwatch.reset()
        while self.stopwatch.time() < time:
            vel = (
                -((self.stopwatch.time() * 10 / time) ** 2)
                + self.stopwatch.time() * (200 / time)
                + 20,
            )

            motor.dc(vel)
            if motor == self.motor_l:
                self.motor_r.hold()
            else:
                self.motor_l.hold()

        self.off_motors()

    def pid_turn(
        self,
        angle,
        pid: PIDValues = PIDValues(
            kp=3,
            ki=0.2,
            kd=8,
        ),
    ):
        """
        Curva com controle PID.
        - Angulo relativo ao eixo do robô.
        - angle positivo: direita, negativo: esquerda
        """
        motor_degrees = self.robot_axis_to_motor_degrees(angle)

        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)

        elapsed_time = 0
        integ = 0.0
        error = 0

        acceptable_degrees = const.PID_TURN_ACCEPTABLE_DEGREES_PERC * motor_degrees
        while abs(self.motor_l.angle()) <= abs(acceptable_degrees):
            motor_angle_average = (self.motor_l.angle() - self.motor_r.angle()) / 2
            prev_error = error
            error = motor_degrees - motor_angle_average

            prop = error * pid.kp
            if abs(error) < 3:
                integ = integ + (error * pid.ki)
            prev_elapsed_time = elapsed_time
            elapsed_time = self.stopwatch.time()

            tempoDecor = elapsed_time - prev_elapsed_time
            tempoDecor = max(tempoDecor, 1)

            deriv = ((error - prev_error) * pid.kd) / tempoDecor

            correction = prop + integ + deriv
            vel = 20 + correction
            if vel < 0:
                vel = min(vel, -20)
            else:
                vel = max(vel, 20)
            self.motor_r.run(-vel)
            self.motor_l.run(vel)

        self.motor_r.hold()
        self.motor_l.hold()

    def simple_walk(self, cm, speed=50, speed_l=None, speed_r=None):
        """Movimentação simples"""
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

    def pid_accelerated_walk(
        self,
        time,
        mode,
        pid: PIDValues = PIDValues(
            kp=3,
            ki=0.02,
            kd=3,
        ),
    ):
        """
        Linha reta acelerada baseada em modos com correção PID entre os motores.

        - Modo 1: termina com velocidade 0
        - Modo 2: termina com velocidade máxima
        - Modo 3: começa com velocidade máxima, termina com velocidade 0
        """

        elapsed_time = 0
        i_share = 0
        error = 0

        self.stopwatch.reset()
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)

        while self.stopwatch.time() < abs(time):

            prev_error = error
            error = self.motor_r.angle() - self.motor_l.angle()
            p_share = error * pid.kp

            if abs(error) < 3:
                i_share = i_share + (error * pid.ki)

            prev_elapsed_time = elapsed_time
            wait(1)
            elapsed_time = self.stopwatch.time()
            d_share = ((error - prev_error) * pid.kd) / (
                elapsed_time - prev_elapsed_time
            )

            pid_correction = p_share + i_share + d_share

            if mode == 1:
                a = 1.0
                b = 1.0
                c = 0.0
            elif mode == 2:
                a = 0.5
                b = 0.5
                c = 0.0
            elif mode == 3:
                a = 0.5
                b = 0.0
                c = 80.0

            vel = (
                -((elapsed_time * a * 20 / abs(time)) ** 2)
                + elapsed_time * (b * 400 / abs(time))
                + (c + 20)
            )
            sign = -1 if time < 0 else 1
            self.motor_l.dc(sign * vel + pid_correction)
            self.motor_r.dc(sign * vel - pid_correction)

        self.motor_l.hold()
        self.motor_r.hold()

    def pid_align(
        self,
        pid: PIDValues = PIDValues(target=30, kp=0.7, ki=0.001, kd=0.3),
    ):
        """
        Alinha usando os dois pares (motor - sensor de cor) e controle PID.
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
            left_error = self.color_l.reflection() - pid.target
            right_error = self.color_r.reflection() - (pid.target * correction_factor)

            left_error_i += left_error
            right_error_i += right_error

            left_error_d = left_error - left_prev_error
            right_error_d = right_error - right_prev_error

            left_prev_error = left_error
            right_prev_error = right_error

            # ev3_print(left_error, left_error_i, left_error_d)
            left_pid_speed = (
                pid.kp * left_error + pid.ki * left_error_i + pid.kd * left_error_d
            )
            right_pid_speed = (
                pid.kp * right_error + pid.ki * right_error_i + pid.kd * right_error_d
            )

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

    def pid_line_grabber(  # pylint: disable=invalid-name
        self,
        vel,
        time,
        sensor: ColorSensor,
        pid: PIDValues = PIDValues(
            target=35,  # medir na linha toda vez
            kp=3.5,
            ki=0.05,
            kd=10,
        ),
    ):
        """
        O robô usa um dos sensores de cor para encontrar a linha e entrar em posição
        ideal antes de iniciar o algoritmo de seguidor de linha.
        """

        error = 0
        i_share = 0.0
        elapsed_time = 0

        self.stopwatch.reset()
        while True:
            prev_error = error
            error = pid.target - sensor.reflection()
            p_share = error * pid.kp

            if abs(error) < 3:
                i_share = (i_share + error) * pid.ki

            prev_elapsed_time = elapsed_time
            wait(1)
            elapsed_time = self.stopwatch.time()
            d_share = ((error - prev_error) * pid.kd) / (
                elapsed_time - prev_elapsed_time
            )

            pid_correction = p_share + i_share + d_share

            pid_sign = 1 if sensor == self.color_l else -1
            self.motor_r.run(vel + (pid_correction * pid_sign))
            self.motor_l.run(vel - (pid_correction * pid_sign))

            if self.stopwatch.time() > time:
                break
        self.off_motors()

    def pid_line_follower_color_id(
        self,
        vel,
        sensor_follow,
        sensor_color,
        array,
        pid: PIDValues = PIDValues(
            target=35,  # medir na linha toda vez
            kp=0.25,
            ki=0.003,
            kd=0.4,
        ),
    ):
        """
        Seguidor de linha + leitor de cores
        """
        # TODO: melhorar docstring
        # TODO: conferir se não faz mais sentido colocar essa função no domain
        error = 0
        i_share = 0.0
        elapsed_time = 0

        valid_colors = [Color.YELLOW, Color.BLUE, Color.RED]

        self.stopwatch.reset()
        while True:
            prev_error = error
            error = pid.target - sensor_follow.reflection()
            p_share = error * pid.kp

            if abs(error) < 3:
                i_share = (i_share + error) * pid.ki

            prev_elapsed_time = elapsed_time
            wait(1)
            elapsed_time = self.stopwatch.time()
            d_share = ((error - prev_error) * pid.kd) / (
                elapsed_time - prev_elapsed_time
            )

            pid_correction = p_share + i_share + d_share

            pid_sign = 1 if sensor_follow == self.color_l else -1

            self.motor_r.dc(vel + pid_correction * pid_sign)
            self.motor_l.dc(vel - pid_correction * pid_sign)

            if (
                sensor_color.color() not in array
                and sensor_color.color() in valid_colors
            ):
                array.append(sensor_color.color())
                valid_colors.remove(sensor_color.color())

            if sensor_color.color() is None:
                break

        self.motor_r.hold()
        self.motor_l.hold()

        return array
