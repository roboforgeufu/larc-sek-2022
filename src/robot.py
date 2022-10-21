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

import constants as const
from utils import PIDValues, between, normalize_color, wait_button_pressed


class Robot:
    """
    Classe que representa um robô genérico.
    """

    def __init__(
        self,
        wheel_diameter: float,
        wheel_distance: float,
        motor_r: Port = None,
        motor_l: Port = None,
        motor_claw: Port = None,
        motor_sensor: Port = None,
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
        debug: bool = False,
        turn_correction: float = 1,
        color_max_value: float = 100,
    ) -> None:
        # Brick EV3
        self.brick = EV3Brick()

        # Medidas do robô
        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance
        self.turn_correction = turn_correction
        self.color_max_value = color_max_value

        # Cronometro
        self.stopwatch = StopWatch()

        # Debug
        self.debug = debug

        # Motores
        if motor_r is not None:
            self.motor_r = Motor(motor_r)
        if motor_l is not None:
            self.motor_l = Motor(motor_l)
        if motor_claw is not None:
            self.motor_claw = Motor(motor_claw)
        if motor_sensor is not None:
            self.motor_sensor = Motor(motor_sensor)

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
            self.ultra_side = UltrasonicSensor(ultra_side)
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
        """
        Grau relativo ao eixo do robô -> grau nas rodas (motores) do robô

        Considera um possível fator de correção
        """
        return (
            axis_degrees
            * (self.wheel_distance / self.wheel_diameter)
            * self.turn_correction
        )

    def cm_to_motor_degrees(self, cm: float):
        """Distância em centímetros -> grau nas rodas (motores) do robô"""
        return cm * (360 / (math.pi * self.wheel_diameter))

    def off_motors(self):
        """Desliga motores de locomoção."""
        self.motor_l.dc(0)
        self.motor_r.dc(0)
        self.motor_l.hold()
        self.motor_r.hold()
        self.motor_l.dc(0)
        self.motor_r.dc(0)

    def ev3_print(self, *args, clear=False, **kwargs):
        """
        Métodos para logs.
        """
        if self.debug:
            if clear:
                wait(10)
                self.brick.screen.clear()
            self.brick.screen.print(*args, **kwargs)
            print(*args, **kwargs)

    def forward_while_same_reflection(
        self,
        speed_r=50,
        speed_l=50,
        reflection_diff=10,
        avoid_obstacles=False,
        pid: PIDValues = PIDValues(
            kp=1,
            ki=0.001,
            kd=1,
        ),
    ):
        """
        Move ambos os motores (de forma individual) até que a intensidade de reflexão
        mude o suficiente (`reflection_diff`)
        """
        # self.ev3_print(self.forward_while_same_reflection.__name__)

        starting_ref_r = self.color_r.reflection()
        starting_ref_l = self.color_l.reflection()

        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)

        motor_error_i = 0
        prev_motor_error = 0

        stopped_l = False
        stopped_r = False
        while not stopped_l or not stopped_r:
            if avoid_obstacles and self.ultra_front.distance() < const.OBSTACLE_DIST:
                self.brick.speaker.beep(100)
                self.pid_walk(10, -50)
                self.pid_turn(90)
                self.motor_l.reset_angle(0)
                self.motor_r.reset_angle(0)
                self.brick.speaker.beep(1000)
                continue

            diff_ref_r = self.color_r.reflection() - starting_ref_r
            diff_ref_l = self.color_l.reflection() - starting_ref_l

            # Controle PID entre os motores
            if (not stopped_l and not stopped_r) and (speed_l == speed_r):
                motor_diff = self.motor_r.angle() - self.motor_l.angle()
                motor_error = motor_diff

                motor_error_i += motor_error
                motor_error_d = motor_error - prev_motor_error
                prev_motor_error = motor_error

                # self.ev3_print(
                #     motor_error,
                #     motor_error_i,
                #     motor_error_d,
                # )
                pid_speed = (
                    pid.kp * motor_error
                    + pid.ki * motor_error_i
                    + pid.kd * motor_error_d
                )
            else:
                pid_speed = 0
            ###

            # self.ev3_print(diff_ref_l, diff_ref_r)

            if abs(diff_ref_r) < reflection_diff:
                self.motor_r.dc(speed_r - pid_speed)
            else:
                if not stopped_l:
                    stopped_l = True
                self.motor_r.hold()

            if abs(diff_ref_l) < reflection_diff:
                self.motor_l.dc(speed_l + pid_speed)
            else:
                if not stopped_r:
                    stopped_r = True
                self.motor_l.hold()
        self.off_motors()

    def simple_turn(
        self, angle, speed=50, look_around_function=None, motor_correction=None
    ):
        """
        Curva simples.


        Caso look_around_function seja passado, retorna uma lista com todas
        as leituras durante a curva.

        A lista retornada contém tuplas com: (valores lidos, motor_r, motor_l).

        motor_correction é um valor de 0 a 1
        O sinal de motor_correction define em qual motor a correção será aplicada:
        - Negativo: motor direito
        - Positivo: motor esquerdo
        """
        dir_sign = 1 if angle > 0 else -1
        speed = abs(speed)

        motor_degrees = self.robot_axis_to_motor_degrees(abs(angle))
        if motor_correction is not None:
            if motor_correction > 0:
                motor_degrees_l = motor_degrees * abs(motor_correction)
                motor_degrees_r = motor_degrees
            else:
                motor_degrees_r = motor_degrees * abs(motor_correction)
                motor_degrees_l = motor_degrees
        else:
            motor_degrees_l = motor_degrees
            motor_degrees_r = motor_degrees

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        reads = []

        while (
            abs(self.motor_l.angle() - initial_angle_l) < motor_degrees_l
            or abs(self.motor_r.angle() - initial_angle_r) < motor_degrees_r
        ):
            if look_around_function is not None:
                reads.append(
                    (
                        look_around_function(),
                        self.motor_r.angle(),
                        self.motor_l.angle(),
                    )
                )

            if abs(self.motor_r.angle() - initial_angle_r) < motor_degrees:
                self.motor_r.dc(dir_sign * -speed)
            else:
                self.motor_r.dc(0)

            if abs(self.motor_l.angle() - initial_angle_l) < motor_degrees:
                self.motor_l.dc(dir_sign * speed)
            else:
                self.motor_l.dc(0)

        self.off_motors()
        return reads

    def one_wheel_turn(self, motor: Motor, angle: int, speed: int):
        # Curva com um motor
        dir_sign = 1 if angle > 0 else -1
        initial_angle = motor.angle()
        while abs(motor.angle() - initial_angle) < self.robot_axis_to_motor_degrees(
            abs(angle) * 2
        ):
            motor.dc(speed * dir_sign)
        self.off_motors()

    def one_wheel_turn_till_color(self, motor: Motor, sensor_color, target_color):
        motor.reset_angle(0)
        vel = 50
        while self.accurate_color(sensor_color.rgb()) != target_color:
            motor.dc(vel)
        self.off_motors()
        return motor.angle()

    def turn_till_color(self, direction, sensor_color, target_color):
        sign = 1 if direction == "right" else -1
        vel = 50
        while self.accurate_color(sensor_color.rgb()) != target_color:
            self.motor_l.dc(sign * vel)
            self.motor_r.dc(sign * (-vel))
        self.off_motors()

    def certify_line_alignment_routine(self, motor: Motor, sensor_color, target_color):
        degrees = self.one_wheel_turn_till_color(motor, sensor_color, target_color)
        self.pid_walk(cm=const.WHEEL_DIAMETER, vel=30)
        motor.reset_angle(0)
        target_motor = self.motor_r if motor == self.motor_l else self.motor_l
        target_motor.run_target(100, -220 + degrees)
        self.line_grabber(vel=20, time=3000, sensor=sensor_color)

    def move_both_to_target(
        self,
        target=None,
        target_l=None,
        target_r=None,
        tolerable_diff=2,
        speed=30,
    ):
        """
        Move os dois motores simultaneamente para um ângulo alvo (que pode ser
        diferente entre eles).

        Suporta curvas!
        """
        if target is not None:
            target_l = target
            target_r = target

        while (
            abs(self.motor_l.angle() - target_l) > tolerable_diff
            or abs(self.motor_r.angle() - target_r) > tolerable_diff
        ):
            if abs(self.motor_l.angle() - target_l) > tolerable_diff:
                dir_sign_l = -1 if (self.motor_l.angle() - target_l) > 0 else 1
                self.motor_l.dc(speed * dir_sign_l)
            else:
                self.motor_l.dc(0)

            if abs(self.motor_r.angle() - target_r) > tolerable_diff:
                dir_sign_r = -1 if (self.motor_r.angle() - target_r) > 0 else 1
                self.motor_r.dc(speed * dir_sign_r)
            else:
                self.motor_r.dc(0)
        self.off_motors()

    def pid_turn(
        self,
        angle,
        mode=1,
        pid: PIDValues = PIDValues(
            kp=0.7,
            ki=0.1,
            kd=0.2,
        ),
    ):
        """
        Curva com controle PID.
        - Angulo relativo ao eixo do robô.
        - Angulo negativo: curva p / esquerda
        - Angulo positivo: curva p / direita
        - Modos(mode):
            - 1: usa o valor dado como ângulo ao redor do eixo do robô
            - 2: usa o valor dado como ângulo no eixo das rodas
        """
        # self.ev3_print(self.pid_turn.__name__)

        if mode == 1:
            motor_degrees = self.robot_axis_to_motor_degrees(angle)
        if mode == 2:
            motor_degrees = angle

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        target_angle_r = initial_angle_r - motor_degrees
        target_angle_l = initial_angle_l + motor_degrees

        # self.ev3_print("INICIAL:", initial_angle_l, initial_angle_r)
        # self.ev3_print("TARGET:", target_angle_l, target_angle_r)

        left_error = target_angle_l - self.motor_l.angle()
        right_error = target_angle_r - self.motor_r.angle()

        left_error_i = 0
        right_error_i = 0

        left_prev_error = left_error
        right_prev_error = right_error

        n = 0
        while (
            int(abs(self.motor_l.angle() - target_angle_l))
            > const.PID_TURN_ACCEPTABLE_DEGREE_DIFF
            or int(abs(self.motor_r.angle() - target_angle_r))
            > const.PID_TURN_ACCEPTABLE_DEGREE_DIFF
        ):
            n += 1
            left_error = target_angle_l - self.motor_l.angle()
            right_error = target_angle_r - self.motor_r.angle()

            if abs(left_error) < 30:
                left_error_i += left_error
            if abs(right_error) < 30:
                right_error_i += right_error

            left_error_d = left_prev_error - left_error
            right_error_d = right_prev_error - right_error

            left_prev_error = left_error
            right_prev_error = right_error

            left_pid_speed = (
                pid.kp * left_error + pid.ki * left_error_i + pid.kd * left_error_d
            )
            right_pid_speed = (
                pid.kp * right_error + pid.ki * right_error_i + pid.kd * right_error_d
            )
            # self.ev3_print(
            #     self.motor_l.angle(),
            #     self.motor_r.angle(),
            #     "|",
            #     left_error,
            #     left_error_i,
            #     left_error_d,
            #     left_pid_speed,
            # )

            # Limitante de velocidade
            left_speed_sign = -1 if left_pid_speed < 0 else 1
            left_pid_speed = min(75, abs(left_pid_speed)) * left_speed_sign
            right_speed_sign = -1 if right_pid_speed < 0 else 1
            right_pid_speed = min(75, abs(right_pid_speed)) * right_speed_sign

            self.motor_l.dc(left_pid_speed)
            self.motor_r.dc(right_pid_speed)

            left_wheel_angle_distance = self.motor_l.angle() - initial_angle_l
            right_wheel_angle_distance = self.motor_r.angle() - initial_angle_r

            # self.ev3_print("C:", self.motor_l.speed(), self.motor_r.speed())
            if (
                abs(self.motor_l.speed()) < const.PID_TURN_MIN_SPEED
                and abs(self.motor_r.speed()) < const.PID_TURN_MIN_SPEED
                and abs(left_wheel_angle_distance) > const.MIN_DEGREES_CURVE_THRESHOLD
                and abs(right_wheel_angle_distance) > const.MIN_DEGREES_CURVE_THRESHOLD
            ):
                break
        self.off_motors()
        # self.ev3_print(n, "| END:", self.motor_l.angle(), self.motor_r.angle())

    def simple_walk(self, cm, speed=50, speed_l=None, speed_r=None):
        """Movimentação simples"""
        dir_sign = 1 if cm > 0 else -1

        speed = abs(speed)
        if speed_l is None:
            speed_l = speed
        if speed_r is None:
            speed_r = speed
        speed_l = abs(speed_l)
        speed_r = abs(speed_r)

        degrees = self.cm_to_motor_degrees(cm)

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        while abs(initial_angle_l - self.motor_l.angle()) < abs(degrees) or abs(
            initial_angle_r - self.motor_r.angle()
        ) < abs(degrees):
            if abs(initial_angle_l - self.motor_l.angle()) < abs(degrees):
                self.motor_l.dc(speed_l * dir_sign)
            else:
                self.motor_l.dc(0)

            if abs(initial_angle_r - self.motor_r.angle()) < abs(degrees):
                self.motor_r.dc(speed_l * dir_sign)
            else:
                self.motor_r.dc(0)
        self.off_motors()

    def pid_walk(
        self,
        cm,
        vel=80,
        pid: PIDValues = PIDValues(
            kp=3,
            ki=0.2,
            kd=8,
        ),
    ):
        """Anda em linha reta com controle PID entre os motores."""

        degrees = self.cm_to_motor_degrees(cm)

        elapsed_time = 0
        i_share = 0
        error = 0
        motor_angle_average = 0
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)
        self.stopwatch.reset()

        while abs(motor_angle_average) < abs(degrees):
            motor_angle_average = (self.motor_l.angle() + self.motor_r.angle()) / 2
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
            self.motor_r.dc(vel - pid_correction)
            self.motor_l.dc(vel + pid_correction)

        self.off_motors()

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
        i_share = 0.0
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

        self.off_motors()

    def pid_align(
        self,
        pid: PIDValues = PIDValues(target=30, kp=2, ki=0.001, kd=0.3),
    ):
        """
        Alinha usando os dois pares (motor - sensor de cor) e controle PID.
        """
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

            # self.ev3_print(left_error, right_error)
            if abs(left_error) <= 7 and abs(right_error) <= 7:
                break
        self.off_motors()

    def line_grabber(self, sensor, time, vel=20, multiplier = 1.5):
        color_reads = []
        num_reads = 10
        wrong_read_perc = 0.5
        color_count_perc = 0.5
        self.stopwatch.reset()
        self.reset_both_motor_angles()
        while True:

            sign = 1 if sensor == self.color_l else -1
            if (
                (self.accurate_color(sensor.rgb()) != Color.WHITE)
                and (self.accurate_color(sensor.rgb()) != Color.BLACK)
                and (self.accurate_color(sensor.rgb()) != "None")
            ):
                sign = sign * (-1)

            color_read = self.accurate_color(sensor.rgb())
            color_reads.append(color_read)
            left_multiplier = multiplier
            right_multiplier = multiplier
            if len(color_reads) == num_reads:
                black_count_perc = (color_reads.count(Color.BLACK)) / num_reads
                white_count_perc = (color_reads.count(Color.WHITE)) / num_reads
                wrong_read_perc = black_count_perc + white_count_perc
                color_count_perc = 1 - wrong_read_perc
                color_reads.clear()

            self.motor_r.dc(vel + (vel * wrong_read_perc * right_multiplier * sign))
            self.motor_l.dc(vel - (vel * color_count_perc * left_multiplier * sign))
            print(
                vel + (vel * wrong_read_perc * right_multiplier * sign),
                vel - (vel * color_count_perc * left_multiplier * sign),
            )

            motor_mean = (self.motor_l.angle() + self.motor_r.angle()) / 2

            if color_read == "None":
                self.off_motors()
                return motor_mean

            if self.stopwatch.time() > time:
                self.off_motors()
                return motor_mean

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

            ref_read = sensor.reflection()

            if self.stopwatch.time() > time:
                break

            if ref_read < 5:
                break

        self.motor_l.hold()
        self.motor_r.hold()

    def line_follower_color_id(self, sensor, vel=50, array=None, break_color="None"):
        color_reads = []
        all_color_reads = []
        num_reads = 10
        wrong_read_perc = 0.5
        color_count_perc = 0.5
        if array is None:
            array = []
        while True:

            sign = 1 if sensor == self.color_l else -1
            if (
                (self.accurate_color(sensor.rgb()) != Color.WHITE)
                and (self.accurate_color(sensor.rgb()) != Color.BLACK)
                and (self.accurate_color(sensor.rgb()) != "None")
            ):
                sign = sign * (-1)

            color_read = self.accurate_color(sensor.rgb())
            color_reads.append(color_read)
            left_multiplier = 0.33
            right_multiplier = 0.33
            if len(color_reads) == num_reads:
                black_count_perc = (color_reads.count(Color.BLACK)) / num_reads
                white_count_perc = (color_reads.count(Color.WHITE)) / num_reads
                green_count_perc = (color_reads.count(Color.GREEN)) / num_reads
                wrong_read_perc = black_count_perc + white_count_perc + green_count_perc
                color_count_perc = 1 - wrong_read_perc
                all_color_reads.extend(color_reads)
                color_reads.clear()

            self.motor_r.dc(vel + (vel * wrong_read_perc * right_multiplier * sign))
            self.motor_l.dc(vel - (vel * color_count_perc * left_multiplier * sign))

            # if color_read not in array and color_read in valid_colors:
            if len(all_color_reads) > 50:
                y_count = all_color_reads.count(Color.YELLOW)
                r_count = all_color_reads.count(Color.RED)
                b_count = all_color_reads.count(Color.BLUE)
                max_count = max(y_count, r_count, b_count)
                if max_count == y_count:
                    if Color.YELLOW not in array:
                        array.append(Color.YELLOW)
                elif max_count == r_count:
                    if Color.RED not in array:
                        array.append(Color.RED)
                elif max_count == b_count:
                    if Color.BLUE not in array:
                        array.append(Color.BLUE)
                all_color_reads.clear()

            if break_color == "None" and sensor == self.color_l:
                if len(array) >= 2:
                    break
            else:
                if color_read == break_color:
                    break

            if color_read == "None":
                break

        self.off_motors()

        return array

    def min_aligner(
        self,
        min_function,
        speed: int = 40,
        max_angle=90,
        acceptable_range=0,
        motor_correction=None,
    ):
        """
        Alinha os motores usando o mínimo de uma função como alvo.

        O argumento `min_function` deve ser a função a ser "minimizada"
        (infra_sensor.distance, por exemplo).
        """
        infra_reads = self.simple_turn(
            -(max_angle / 2),
            speed=speed,
            look_around_function=min_function,
            motor_correction=motor_correction,
        )
        second_reads = self.simple_turn(
            max_angle,
            speed=speed,
            look_around_function=min_function,
            motor_correction=motor_correction,
        )
        infra_reads.extend(second_reads)

        min_read = min(i_read for i_read, _, _ in infra_reads)
        close_reads = [
            read
            for read in infra_reads
            if read[0] in range(min_read, min_read + acceptable_range + 1)
        ]

        # self.ev3_print(
        #     "=========", min_read, min_read + acceptable_range + 1, "========="
        # )
        # for read in close_reads:
        #     self.ev3_print("CLOSE MN ALGN:", read)
        # self.ev3_print("==================")

        motor_r_mean = sum(motor_r_angle for _, motor_r_angle, _ in close_reads) / len(
            close_reads
        )
        motor_l_mean = sum(motor_l_angle for _, _, motor_l_angle in close_reads) / len(
            close_reads
        )

        self.move_both_to_target(target_l=motor_l_mean, target_r=motor_r_mean)

    def pid_wall_follower(
        self,
        speed=30,
        front_sensor=None,
        side_dist=const.WALL_FOLLOWER_SIDE_DIST,
        pid: PIDValues = PIDValues(
            kp=0.8,
            ki=0.001,
            kd=0.5,
        ),
        max_cm=None,
    ):
        """Seguidor de parede com controle PID simples."""
        self.ev3_print(self.pid_wall_follower.__name__)

        if front_sensor is None:
            front_sensor = self.ultra_front

        initial_time = self.stopwatch.time()

        initial_angle_r = self.motor_r.angle()
        initial_angle_l = self.motor_l.angle()

        motor_error_i = 0
        prev_motor_error = 0

        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)

        while True:
            motor_diff = self.motor_r.angle() - self.motor_l.angle()

            dist_diff = self.infra_side.distance() - side_dist

            motor_error = motor_diff + (20 * dist_diff)
            motor_error_i += motor_error
            motor_error_d = motor_error - prev_motor_error
            prev_motor_error = motor_error

            pid_speed = (
                pid.kp * motor_error + pid.ki * motor_error_i + pid.kd * motor_error_d
            )

            # self.ev3_print("IR dff:", dist_diff, "|", motor_error)

            self.ev3_print(
                "PID_WLL:",
                motor_error,
                motor_error_i,
                motor_error_d,
            )

            # self.ev3_print("WLFLW t:", self.stopwatch.time() - initial_time)

            # Condições de parada
            if side_dist >= 5 and self.infra_side.distance() == 0:
                self.brick.speaker.beep(700)
                self.brick.speaker.beep(100)
                return_value = 4
                break
            if (
                abs(motor_error) > 100
                and abs(motor_error_d) > 60
                and (
                    self.stopwatch.time() - initial_time
                    > const.WALL_FOLLOWER_THRESHOLD_TIME
                )
            ):
                return_value = 1
                break

            if max_cm is not None:
                self.ev3_print(
                    "MAX_CM_WLL:",
                    abs(self.motor_l.angle() - initial_angle_l),
                    abs(self.motor_r.angle() - initial_angle_r),
                )

            if max_cm is not None and (
                abs(self.motor_l.angle() - initial_angle_l)
                > self.cm_to_motor_degrees(max_cm)
                and abs(self.motor_r.angle() - initial_angle_r)
                > self.cm_to_motor_degrees(max_cm)
            ):
                return_value = 1
                break
            if front_sensor.distance() < 100:
                return_value = 2
                break
            if (
                self.color_l.reflection() < const.COL_REFLECTION_HOLE_MAX
                and self.color_r.reflection() < const.COL_REFLECTION_HOLE_MAX
            ):
                return_value = 3
                break

            # Movimentação
            if self.color_l.reflection() >= const.COL_REFLECTION_HOLE_MAX:
                self.motor_l.dc(speed + pid_speed)
            else:
                self.motor_l.dc(0)

            if self.color_r.reflection() >= const.COL_REFLECTION_HOLE_MAX:
                self.motor_r.dc(speed - pid_speed)
            else:
                self.motor_r.dc(0)

        self.off_motors()
        final_diff = self.motor_r.angle() - self.motor_l.angle()
        # self.ev3_print("final_diff:", final_diff)

        self.move_both_to_target(
            target_r=self.motor_r.angle() - final_diff / 2,
            target_l=self.motor_l.angle() + final_diff / 2,
        )

        return return_value

    def move_to_distance(
        self,
        distance: float,
        sensor: UltrasonicSensor,
        pid=PIDValues(kp=1, ki=0.0001, kd=0.01),
        turning=0,
        max_cm=None,
        safe_max_read=None,
    ):
        """
        Se move até ler determinada distância com o sensor dado.

        - turning é um valor percentual (0 a 1) positivo ou negativo que
        representa o quanto a força será diferente entre dois motores, a fim de
        fazer uma curva.
        - single_motor é um motor opcional caso queira executar o movimento apenas com o motor
        desejado. Se não for passado, os dois motores básicos são usados por padrão.
        """
        # self.ev3_print(self.move_to_distance.__name__, distance)

        if max_cm is not None:
            max_motor_degrees = self.cm_to_motor_degrees(max_cm)
        else:
            max_motor_degrees = 0

        initial_degrees_l = self.motor_l.angle()
        initial_degrees_r = self.motor_r.angle()

        initial_time = self.stopwatch.time()

        diff = sensor.distance() - distance
        diff_i = 0
        prev_diff = diff
        while abs(diff) >= 1 and (
            safe_max_read is None or abs(sensor.distance()) < safe_max_read
        ):
            diff = sensor.distance() - distance
            diff_i += diff
            diff_d = diff - prev_diff

            pid_speed = diff * pid.kp + diff_i * pid.ki + diff_d * pid.kd

            # self.ev3_print(diff, diff_i, diff_d, pid_speed)
            if abs(pid_speed) < 10:
                break

            self.motor_l.dc(pid_speed * (1 + turning))
            self.motor_r.dc(pid_speed * (1 - turning))

            # self.ev3_print(
            #     abs(initial_degrees_l - self.motor_l.angle()),
            #     abs(initial_degrees_r - self.motor_r.angle()),
            # )
            if max_cm is not None and (
                abs(initial_degrees_l - self.motor_l.angle()) > max_motor_degrees
                or abs(initial_degrees_r - self.motor_r.angle()) > max_motor_degrees
            ):
                break

            # self.ev3_print("MV_DIST:", self.motor_l.speed(), self.motor_r.speed())
            if (
                abs(self.motor_l.speed()) < const.PID_TURN_MIN_SPEED
                and abs(self.motor_r.speed()) < const.PID_TURN_MIN_SPEED
                and (
                    self.stopwatch.time() - initial_time
                    > const.MV_TO_DIST_THRESHOLD_TIME
                )
            ):
                break

        # self.ev3_print(sensor.distance())
        self.off_motors()

    def hole_measurement(
        self,
        vel=50,
        pid: PIDValues = PIDValues(
            target=20,
            kp=1,
            ki=0.1,
            kd=3,
        ),
    ):
        pid.kp = 3
        pid.ki = 0.1
        pid.kd = 5

        elapsed_time = 0
        i_share = 0
        error = 0
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)
        self.stopwatch.reset()
        while self.infra_side.distance() > pid.target:

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
            self.motor_r.dc(vel - pid_correction)
            self.motor_l.dc(vel + pid_correction)
        self.off_motors()

        WHEEL_LENGTH = (
            const.WHEEL_DIAMETER * math.pi
        )  # 360 graus = 1 rotacao; 1 rotacao = 17.3cm
        degrees = (self.motor_l.angle() + self.motor_r.angle()) / 2
        return (degrees / 360) * WHEEL_LENGTH

    def duct_measurement(
        self,
        vel=50,
        pid: PIDValues = PIDValues(
            target=50,
            kp=3,
            ki=0.1,
            kd=5,
        ),
        color_check_color=None,
        color_check_sensor=None,
    ):

        elapsed_time = 0
        i_share = 0
        error = 0
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)
        self.stopwatch.reset()
        while self.infra_side.distance() < pid.target and (
            color_check_color is None
            or self.accurate_color(color_check_sensor.rgb()) != color_check_color
        ):

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

            self.motor_r.dc(vel - pid_correction)
            self.motor_l.dc(vel + pid_correction)

        self.motor_l.hold()
        self.motor_r.hold()

        WHEEL_LENGTH = (
            const.WHEEL_DIAMETER * math.pi
        )  # 360 graus = 1 rotacao; 1 rotacao = 17.3cm
        degrees = (self.motor_l.angle() + self.motor_r.angle()) / 2
        # erro medio de medida -> 80 graus
        print(degrees)
        return (degrees / 360) * WHEEL_LENGTH

    def duct_measurement_new(
        self,
        vel=50,
        color_check_color=None,
        color_check_sensor=None,
    ):
        color_reads = []
        num_reads = 10
        wrong_read_perc = 0.5
        color_count_perc = 0.5
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)
        while self.get_average_reading(self.infra_side.distance) < 50 and (
            color_check_color is None
            or self.accurate_color(color_check_sensor.rgb()) != color_check_color
        ):
            sign = 1 if color_check_sensor == self.color_l else -1
            if (
                (self.accurate_color(color_check_sensor.rgb()) != Color.WHITE)
                and (self.accurate_color(color_check_sensor.rgb()) != Color.BLACK)
                and (self.accurate_color(color_check_sensor.rgb()) != "None")
            ):
                sign = sign * (-1)

            color_read = self.accurate_color(color_check_sensor.rgb())
            color_reads.append(color_read)
            left_multiplier = 0.33
            right_multiplier = 0.33
            if len(color_reads) == num_reads:
                black_count_perc = (color_reads.count(Color.BLACK)) / num_reads
                white_count_perc = (color_reads.count(Color.WHITE)) / num_reads
                wrong_read_perc = black_count_perc + white_count_perc
                color_count_perc = 1 - wrong_read_perc
                color_reads.clear()

            self.motor_r.dc(vel + (vel * wrong_read_perc * right_multiplier * sign))
            self.motor_l.dc(vel - (vel * color_count_perc * left_multiplier * sign))

        self.off_motors()

        WHEEL_LENGTH = (
            const.WHEEL_DIAMETER * math.pi
        )  # 360 graus = 1 rotacao; 1 rotacao = 17.3cm
        degrees = (self.motor_l.angle() + self.motor_r.angle()) / 2
        # erro medio de medida -> 80 graus
        # print(degrees)
        return (degrees / 360) * WHEEL_LENGTH

    def walk_to_hole(
        self,
        vel=50,
        mode=1,
        pid: PIDValues = PIDValues(
            kp=1,
            ki=0.1,
            kd=3,
        ),
        color_check_color=None,
        color_check_sensor=None,
    ):
        """
        Modo 1: ré até deixar de ver o buraco
        Modo 2: pra frente até ver o buraco
        Modo 3: frente até deixar de ver (!p/ terra)
        Modo 4: ré até ver (!p/ terra)
        """
        pid.kp = 3
        pid.ki = 0.1
        pid.kd = 5

        elapsed_time = 0
        i_share = 0.0
        error = 0
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)
        self.stopwatch.reset()

        if mode == 1:
            while self.infra_side.distance() > const.WALL_SEEN_DIST and (
                color_check_color is None
                or self.accurate_color(color_check_sensor.rgb()) != color_check_color
            ):
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
                self.motor_r.dc(-(vel + pid_correction))
                self.motor_l.dc(-(vel - pid_correction))

        elif mode == 2:
            while self.infra_side.distance() < const.WALL_SEEN_DIST and (
                color_check_color is None
                or self.accurate_color(color_check_sensor.rgb()) != color_check_color
            ):
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
                self.motor_r.dc(vel - pid_correction)
                self.motor_l.dc(vel + pid_correction)

        elif mode == 3:
            while self.infra_side.distance() > const.WALL_SEEN_DIST and (
                color_check_color is None
                or self.accurate_color(color_check_sensor.rgb()) != color_check_color
            ):
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

                bias = 1
                if self.accurate_color(color_check_sensor.rgb()) == Color.BLACK:
                    bias = 2
                    self.motor_r.dc(bias * vel)
                    self.motor_l.dc(vel)

                self.motor_r.dc(vel - pid_correction)
                self.motor_l.dc(vel + pid_correction)

        elif mode == 4:
            while self.infra_side.distance() < const.WALL_SEEN_DIST and (
                color_check_color is None
                or self.accurate_color(color_check_sensor.rgb()) != color_check_color
            ):
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
                self.motor_r.dc(-(vel + pid_correction))
                self.motor_l.dc(-(vel - pid_correction))

        self.off_motors()

    def accurate_color(self, rgb_tuple):
        """
        Processamento de cor pra evitar os erros da leitura padrão.
        """
        red_value = rgb_tuple[0]
        green_value = rgb_tuple[1]
        blue_value = rgb_tuple[2]

        red_normalized_value = normalize_color(red_value, self.color_max_value)
        green_normalized_value = normalize_color(green_value, self.color_max_value)
        blue_normalized_value = normalize_color(blue_value, self.color_max_value)

        if (
            between(red_normalized_value, 0.11, 0.15)
            and between(green_normalized_value, 0.36, 0.44)
            and between(blue_normalized_value, 0.48, 0.72)
        ):
            return Color.BLUE
        elif (
            green_normalized_value != 0
            and red_normalized_value / green_normalized_value <= 0.5
            and blue_normalized_value / green_normalized_value <= 0.5
        ):
            return Color.GREEN
        elif (
            between(red_normalized_value, 0.71, 0.8)
            and between(green_normalized_value, 0.46, 0.5)
            and between(blue_normalized_value, 0.07, 0.14)
        ):
            return Color.YELLOW
        elif (
            red_normalized_value == 0
            and green_normalized_value == 0
            and blue_normalized_value == 0
        ):
            return "None"
        elif (
            between(red_normalized_value, 0.7, 1)
            and between(green_normalized_value, 0.7, 1)
            and between(blue_normalized_value, 0.7, 1)
        ):
            return Color.WHITE
        elif (
            between(red_normalized_value, 0.57, 0.73)
            and between(green_normalized_value, 0.05, 0.07)
            and between(blue_normalized_value, 0, 0.05)
        ):
            return Color.RED
        else:
            return Color.BLACK

    def walk_till_duct(self, vel=50, color_check_color=None, color_check_sensor=None):
        color_reads = []
        num_reads = 10
        wrong_read_perc = 0.5
        color_count_perc = 0.5
        while self.get_average_reading(self.infra_side.distance) > 50 and (
            color_check_color is None
            or self.accurate_color(color_check_sensor.rgb()) != color_check_color
        ):
            sign = 1 if color_check_sensor == self.color_l else -1
            if (
                (self.accurate_color(color_check_sensor.rgb()) != Color.WHITE)
                and (self.accurate_color(color_check_sensor.rgb()) != Color.BLACK)
                and (self.accurate_color(color_check_sensor.rgb()) != "None")
            ):
                sign = sign * (-1)

            color_read = self.accurate_color(color_check_sensor.rgb())
            color_reads.append(color_read)
            left_multiplier = 0.33
            right_multiplier = 0.33
            if len(color_reads) == num_reads:
                black_count_perc = (color_reads.count(Color.BLACK)) / num_reads
                white_count_perc = (color_reads.count(Color.WHITE)) / num_reads
                wrong_read_perc = black_count_perc + white_count_perc
                color_count_perc = 1 - wrong_read_perc
                color_reads.clear()

            self.motor_r.dc(vel + (vel * wrong_read_perc * right_multiplier * sign))
            self.motor_l.dc(vel - (vel * color_count_perc * left_multiplier * sign))

        self.off_motors()

    def get_average_reading(self, sensor_func, num_reads=100):
        measures = []
        for _ in range(num_reads):
            measures.append(sensor_func())
        mean = sum(measures) / len(measures)
        return mean

    def move_until_end_of_duct(self, speed=25, inverted=False, num_reads=50):
        print("Moving until end of duct")
        sign = 1 if not inverted else -1
        while (
            self.get_average_reading(self.ultra_front.distance, num_reads=num_reads)
            < const.DUCT_ENDS_US_DIFF
        ):
            self.motor_l.dc(sign * speed)
            self.motor_r.dc(sign * -speed)
        self.off_motors()

    def move_until_beginning_of_duct(
        self, speed=25, inverted=False, num_reads=100, time_limit=5000
    ):
        print("Moving until beginning of duct")
        self.stopwatch.reset()
        motor_mean = 0
        sign = 1 if not inverted else -1
        while (
            self.get_average_reading(self.ultra_front.distance, num_reads=num_reads)
            > const.DUCT_ENDS_US_DIFF
            and self.stopwatch.time() < time_limit
            and motor_mean < 150
        ):
            self.motor_l.dc(sign * speed)
            self.motor_r.dc(sign * -speed)
            motor_mean = (abs(self.motor_l.angle()) + abs(self.motor_r.angle())) / 2
        self.off_motors()

    def reset_both_motor_angles(self):
        self.motor_l.reset_angle(0)
        self.motor_r.reset_angle(0)

    def black_line_alignment_routine(self):
        self.forward_while_same_reflection(speed_r=-40, speed_l=-40)
        self.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))
        self.pid_walk(cm=5, vel=-40)
        self.forward_while_same_reflection()
        self.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))
        self.pid_walk(cm=30, vel=-40)
        self.pid_turn(-90)
        self.forward_while_same_reflection()
        self.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))

    def leaves_duct_at_correct_place(self):
        self.pid_walk(cm=40, vel=-30)
        self.pid_turn(90)
        self.forward_while_same_reflection()
        self.pid_align(PIDValues(target=30, kp=1.2, ki=0.002, kd=0.3))
        self.pid_walk(cm=17, vel=-30)
        self.motor_claw.run_target(300, -20)
