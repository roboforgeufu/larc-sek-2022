import constants as const
from robot import Robot
from utils import wait_button_pressed


def gas_duct_routine(robot: Robot):
    robot.forward_while_same_reflection(reflection_diff=15)
    robot.pid_walk(5, -80)
    robot.pid_turn(-90)

    robot.move_to_distance(const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front)
    robot.pid_turn(-90)

    robot.min_aligner(min_function=robot.infra_side.distance)

    while True:
        wall_flw_value = robot.pid_wall_follower(front_sensor=robot.ultra_front)
        if wall_flw_value == 1:
            duct_turn_routine(robot)
        elif wall_flw_value == 2:
            robot.move_to_distance(
                const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front
            )
            robot.pid_turn(-90)
        else:
            break
        robot.min_aligner(min_function=robot.infra_side.distance)
    robot.off_motors()


def duct_turn_routine(robot: Robot, speed=const.SEARCH_WALL_SPEED):
    robot.ev3_print(duct_turn_routine.__name__)
    robot.brick.speaker.beep()
    # wait_button_pressed(robot.brick)
    robot.pid_walk(17)
    robot.pid_turn(90)

    robot.ev3_print("infra_side:", robot.infra_side.distance())
    while robot.infra_side.distance() > const.WALL_SEEN_DIST:
        robot.ev3_print("infra_side:", robot.infra_side.distance())
        robot.motor_l.dc(speed)
        robot.motor_r.dc(speed)

    robot.off_motors()
    robot.pid_walk(3)
    robot.brick.speaker.beep()
    # wait_button_pressed(robot.brick)


def wall_following_turn(
    robot: Robot, high_speed=40, low_speed=15, flw_distance=7, max_turning_angle=500
):
    """
    Faz uma curva pra "dentro" enquanto seguindo a parede.

    Feita pensando em ser usada em conjunto com a `pid_wall_follower`.
    """
    robot.ev3_print(robot.wall_following_turn.__name__)

    initial_turning_angle = 0
    while True:
        robot.ev3_print(robot.infra_side.distance())
        robot.motor_l.dc(high_speed)
        if robot.infra_side.distance() > flw_distance:
            # Perdeu a parede
            # Desacelera um dos motores pra fazer curva
            robot.motor_r.dc(low_speed)
            if initial_turning_angle == 0:
                # Se ainda não estava "contando" angulos da curva
                # A partir daqui, passa a contar (guarda o inicial)
                initial_turning_angle = robot.motor_l.angle()
            elif robot.motor_l.angle() - initial_turning_angle > max_turning_angle:
                # Já estava contando angulos da curva (initial_turning_angle != 0),
                # e o robô excedeu o limite dado
                # Pode terminar a operação
                break
            else:
                # Apenas para logs (durante a curva)
                # robot.ev3_print(robot.motor_l.angle() - initial_turning_angle)
                pass
        else:
            # Mantem os dois motores na mesma velocidade
            robot.motor_r.dc(high_speed)

    while robot.infra_side.distance() > 20:
        robot.ev3_print(robot.infra_side.distance())
        # Vai pra frente enquanto não ver parede só pra garantir que vai
        # terminar com o sensor vendo ela.
        robot.motor_l.dc(high_speed)
        robot.motor_r.dc(high_speed)
        robot.off_motors()
