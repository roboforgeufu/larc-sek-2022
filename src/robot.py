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
from utils import PIDValues, wait_button_pressed


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
    ) -> None:
        # Brick EV3
        self.brick = EV3Brick()

        # Medidas do robô
        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance
        self.turn_correction = turn_correction

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
    ):
        """
        Move ambos os motores (de forma individual) até que a intensidade de reflexão
        mude o suficiente (`reflection_diff`)
        """
        self.ev3_print(self.forward_while_same_reflection.__name__)

        starting_ref_r = self.color_r.reflection()
        starting_ref_l = self.color_l.reflection()

        stopped_l = False
        stopped_r = False
        while not stopped_l or not stopped_r:
            diff_ref_r = self.color_r.reflection() - starting_ref_r
            diff_ref_l = self.color_l.reflection() - starting_ref_l

            self.ev3_print(diff_ref_l, diff_ref_r)

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
        self.off_motors()

    def simple_turn(self, angle, speed=50, look_around_function=None):
        """
        Curva simples.


        Caso look_around_function seja passado, retorna uma lista com todas
        as leituras durante a curva.

        A lista retornada contém tuplas com: (valores lidos, motor_r, motor_l).
        """
        dir_sign = 1 if angle > 0 else -1
        speed = abs(speed)

        motor_degrees = self.robot_axis_to_motor_degrees(abs(angle))

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        reads = []

        while (
            abs(self.motor_l.angle() - initial_angle_l) < motor_degrees
            or abs(self.motor_r.angle() - initial_angle_r) < motor_degrees
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

    def one_wheel_turn(self, time, motor: Motor):
        """
        Curva temporizada com uma roda.
        """
        self.stopwatch.reset()
        while self.stopwatch.time() < time:
            vel = (
                -((self.stopwatch.time() * 10 / time) ** 2)
                + self.stopwatch.time() * (200 / time)
                + 20
            )

            motor.dc(vel)
            if motor == self.motor_l:
                self.motor_r.hold()
            else:
                self.motor_l.hold()

        self.off_motors()

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
            kp=2,
            ki=0,
            kd=0.5,
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
        self.ev3_print(self.pid_turn.__name__)

        if mode == 1:
            motor_degrees = self.robot_axis_to_motor_degrees(angle)
        if mode == 2:
            motor_degrees = angle

        initial_angle_l = self.motor_l.angle()
        initial_angle_r = self.motor_r.angle()

        target_angle_r = initial_angle_r - motor_degrees
        target_angle_l = initial_angle_l + motor_degrees

        self.ev3_print("INICIAL:", initial_angle_l, initial_angle_r)
        self.ev3_print("TARGET:", target_angle_l, target_angle_r)

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
            self.ev3_print(
                self.motor_l.angle(),
                self.motor_r.angle(),
                "|",
                left_error,
                left_error_i,
                left_error_d,
                left_pid_speed,
            )

            # Limitante de velocidade
            left_speed_sign = -1 if left_pid_speed < 0 else 1
            left_pid_speed = min(75, abs(left_pid_speed)) * left_speed_sign
            right_speed_sign = -1 if right_pid_speed < 0 else 1
            right_pid_speed = min(75, abs(right_pid_speed)) * right_speed_sign

            self.motor_l.dc(left_pid_speed)
            self.motor_r.dc(right_pid_speed)

        self.off_motors()
        self.ev3_print(n, "| END:", self.motor_l.angle(), self.motor_r.angle())

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

    def min_aligner(self, min_function, speed: int = 40, max_angle=90):
        """
        Alinha os motores usando o mínimo de uma função como alvo.

        O argumento `min_function` deve ser a função a ser "minimizada"
        (infra_sensor.distance, por exemplo).
        """
        infra_reads = self.simple_turn(
            -(max_angle / 2), speed=speed, look_around_function=min_function
        )
        second_reads = self.simple_turn(
            max_angle, speed=speed, look_around_function=min_function
        )
        infra_reads.extend(second_reads)

        min_read = min(i_read for i_read, _, _ in infra_reads)
        close_reads = [read for read in infra_reads if read[0] == min_read]

        motor_r_mean = sum(motor_r_angle for _, motor_r_angle, _ in close_reads) / len(
            close_reads
        )
        motor_l_mean = sum(motor_l_angle for _, _, motor_l_angle in close_reads) / len(
            close_reads
        )

        self.move_both_to_target(target_l=motor_l_mean, target_r=motor_r_mean)

    def pid_wall_follower(
        self,
        speed=50,
        front_sensor=None,
        side_dist=const.WALL_FOLLOWER_SIDE_DIST,
        pid: PIDValues = PIDValues(
            kp=0.8,
            ki=0.001,
            kd=1,
        ),
    ):
        """Seguidor de parede com controle PID simples."""
        self.ev3_print(self.pid_wall_follower.__name__)

        if front_sensor is None:
            front_sensor = self.ultra_front_r

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

            self.ev3_print(
                motor_error,
                motor_error_i,
                motor_error_d,
                "| COL_REFL:",
                self.color_l.reflection(),
                self.color_r.reflection(),
            )

            # Condições de parada
            if abs(motor_error) > 100 and abs(motor_error_d) > 60:
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
        return return_value

    def move_to_distance(
        self,
        distance: float,
        sensor: UltrasonicSensor,
        pid=PIDValues(kp=1, ki=0.0001, kd=0.01),
        turning=0,
        single_motor: Motor = None,
    ):
        """
        Se move até ler determinada distância com o sensor dado.

        - turning é um valor percentual (0 a 1) positivo ou negativo que
        representa o quanto a força será diferente entre dois motores, a fim de
        fazer uma curva.
        - single_motor é um motor opcional caso queira executar o movimento apenas com o motor
        desejado. Se não for passado, os dois motores básicos são usados por padrão.
        """
        self.ev3_print(self.move_to_distance.__name__)

        diff = sensor.distance() - distance
        diff_i = 0
        prev_diff = diff
        while abs(diff) >= 1:
            diff = sensor.distance() - distance
            diff_i += diff
            diff_d = diff - prev_diff
            # ev3_print(diff, diff_i, diff_d, ev3=self.brick)

            pid_speed = diff * pid.kp + diff_i * pid.ki + diff_d * pid.kd

            if abs(pid_speed) < 10:
                break

            self.ev3_print(pid_speed)
            if single_motor is None:
                self.motor_l.dc(pid_speed * (1 + turning))
                self.motor_r.dc(pid_speed * (1 - turning))
            else:
                single_motor.dc(pid_speed)
        self.ev3_print(sensor.distance())
        self.off_motors()

    def align_front_wall(self):
        self.ultra_front_r.distance()
        self.ultra_front_l.distance()
        dist_right = self.ultra_front_r.distance()
        dist_left = self.ultra_front_l.distance()

        if dist_right > dist_left:
            # O sensor da esquerda está mais perto
            sensor_far = self.ultra_front_r
            motor_far = self.motor_r
            sensor_close = self.ultra_front_l
            motor_close = self.motor_l
        else:
            # O sensor da direita está mais perto
            sensor_far = self.ultra_front_l
            motor_far = self.motor_l
            sensor_close = self.ultra_front_r
            motor_close = self.motor_r

        # sensor_far.distance(silent=True)  # silencia sensor que tá mais longe
        self.move_to_distance(60, sensor=sensor_close)
        # sensor_close.distance(silent=True)  # silencia sensor mais perto
        self.move_to_distance(60, sensor=sensor_far, single_motor=motor_far)
        # sensor_far.distance(silent=True)  # silencia sensor mais longe
        self.move_to_distance(60, sensor=sensor_close, single_motor=motor_close)

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

    def walk_to_hole(
        self,
        vel=50,
        mode=1,
        pid: PIDValues = PIDValues(
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

        if mode == 1:
            while self.infra_side.distance() > 25:
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
            while self.infra_side.distance() < 25:
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
