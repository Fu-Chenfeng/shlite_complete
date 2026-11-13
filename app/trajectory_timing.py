"""Scale replay timing without modifying the recorded trajectory or its path."""
import copy
import math

import rospy


def scale_trajectory_speed(trajectory, speed=1.0):
    if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed <= 0:
        raise ValueError('Replay speed must be a finite positive number')
    result = copy.deepcopy(trajectory)
    for point in result.joint_trajectory.points:
        point.time_from_start = rospy.Duration.from_sec(point.time_from_start.to_sec() / speed)
        point.velocities = [value * speed for value in point.velocities]
        point.accelerations = [value * speed * speed for value in point.accelerations]
    return result
