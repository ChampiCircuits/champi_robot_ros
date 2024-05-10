from geometry_msgs.msg import Twist


# PID controller parameters
kp = 0.5  # Proportional gain
ki = 0.1  # Integral gain
kd = 0.2  # Derivative gain

def go_to_pose(target_pose, robot_pose):
    if robot_pose is None:
        # Robot pose not available yet
        return

    # Compute error
    error_x = target_pose.position.x - robot_pose.position.x
    error_y = target_pose.position.y - robot_pose.position.y

    # PID control
    cmd_vel = Twist()
    cmd_vel.linear.x = kp * error_x
    cmd_vel.linear.y = kp * error_y

    return cmd_vel
