#!/usr/bin/env python3
"""Load the configured environment into the real MoveIt planning scene."""
import math

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

from project_config import load_config, project_path, object_pose, world_model_names


def box_pose(values):
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values[:3]
    roll, pitch, yaw = (value / 2 for value in values[3:])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    q = pose.pose.orientation
    q.x, q.y = sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy
    q.z, q.w = cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy
    return pose


def main():
    config = load_config()
    present = world_model_names(config)
    rospy.init_node('add_cell_obstacles')
    moveit_commander.roscpp_initialize([])
    try:
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        for definition in config['environment']:
            if definition['enabled'] and definition['name'] in present:
                scene.add_box(definition['name'], box_pose(definition['pose']), size=definition['size'])
            else:
                scene.remove_world_object(definition['name'])
        obj = config['object']
        if obj['enabled'] and obj['add_to_real_scene']:
            if obj.get('collision_mesh', False):
                scene.add_mesh('rect_object', box_pose(object_pose(config)),
                               str(project_path(obj['mesh'])), size=tuple(obj['mesh_scale']))
            else:
                scene.add_box('rect_object', box_pose(object_pose(config)), size=obj['collision_size'])
        else:
            scene.remove_world_object('rect_object')
        print('Loaded environment from %s into MoveIt planning scene.' % config['simulation']['world'])
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
