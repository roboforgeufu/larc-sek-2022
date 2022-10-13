import constants as const
from robot import Robot


def gas_duct_routine(robot: Robot):
    robot.forward_while_same_reflection(reflection_diff=15)
    robot.simple_walk(-5)
    robot.pid_turn(-80)

    robot.move_to_distance(const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front)
    robot.pid_turn(-90)

    robot.min_aligner(min_function=robot.infra_side.distance)

    while True:
        wall_flw_value = robot.pid_wall_follower(front_sensor=robot.ultra_front)
        if wall_flw_value == 1:
            robot.wall_following_turn(high_speed=40, low_speed=20)
            robot.pid_walk(7)
        elif wall_flw_value == 2:
            robot.move_to_distance(
                const.WALL_FOLLOWER_FRONT_DIST, sensor=robot.ultra_front
            )
            robot.pid_turn(-90)
        else:
            break
        robot.min_aligner(min_function=robot.infra_side.distance)
    robot.off_motors()
