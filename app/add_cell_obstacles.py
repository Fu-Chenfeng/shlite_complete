#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

rospy.init_node("add_cell_obstacles")
moveit_commander.roscpp_initialize([])

scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2.0)

frame = "world"

def add_box(name, size, xyz):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = xyz[0]
    pose.pose.position.y = xyz[1]
    pose.pose.position.z = xyz[2]
    pose.pose.orientation.w = 1.0
    scene.add_box(name, pose, size=size)
    rospy.sleep(0.5)

# Same as your Gazebo world/SDF:
add_box("ground_plane_user", (0.9, 0.66, 0.002), (0.14, 0.0, -0.001))
add_box("cell_plane_user",   (0.9, 0.66, 0.002), (0.14, 0.0, 0.951))
add_box("back_plane_user",   (0.002, 0.66, 0.95), (-0.31, 0.0, 0.475))
add_box("side_plane_right",  (0.9, 0.002, 0.95), (0.14, -0.33, 0.475))
add_box("side_plane_left",   (0.9, 0.002, 0.95), (0.14, 0.33, 0.475))

print("Added Gazebo-matching obstacles to MoveIt planning scene.")
rospy.sleep(2.0)
