from robot import Robot


def gas_duct_routine(robot: Robot):
    robot.forward_while_same_reflection(reflection_diff=5)
    robot.simple_walk(7, -50)
    robot.pid_turn(-90)

    robot.move_to_distance(110, sensor=robot.ultra_front)
    robot.pid_turn(-90)

    robot.wall_aligner()

    while True:
        wall_flw_value = robot.pid_wall_follower(front_sensor=robot.ultra_front)
        if wall_flw_value == 1:
            robot.wall_following_turn(high_speed=40, low_speed=20)
            robot.pid_walk(7)
        elif wall_flw_value == 2:
            robot.move_to_distance(110, sensor=robot.ultra_front)
            robot.pid_turn(-90)
        else:
            break
        robot.wall_aligner()
    robot.off_motors()
