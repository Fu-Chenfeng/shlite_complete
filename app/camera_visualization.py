#!/usr/bin/env python3
"""Show camera cones in RViz and Gazebo when a camera tour is requested."""
import argparse
import json
from pathlib import Path

GEOMETRY_TOPIC = '/camera_scan/geometry'
MARKER_TOPIC = '/camera_scan/view_cones'
DISPLAY_NAME = 'Camera views'


def configure_rviz(path):
    """Add our display without replacing the vendor's robot/planning displays."""
    import yaml
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    displays = config['Visualization Manager']['Displays']
    displays[:] = [display for display in displays if display.get('Name') != DISPLAY_NAME]
    displays.append({'Class': 'rviz/MarkerArray', 'Name': DISPLAY_NAME,
                     'Enabled': True, 'Marker Topic': MARKER_TOPIC, 'Queue Size': 10})
    path.write_text(yaml.safe_dump(config, sort_keys=False))


def publish_geometry(geometry):
    """Send the actual planned targets to the persistent scene node."""
    import time
    import rospy
    from std_msgs.msg import String
    publisher = rospy.Publisher(GEOMETRY_TOPIC, String, queue_size=1, latch=True)
    deadline = time.monotonic() + 1.0
    while not publisher.get_num_connections() and time.monotonic() < deadline and not rospy.is_shutdown():
        time.sleep(.02)
    publisher.publish(String(data=json.dumps(geometry)))
    return publisher  # Keep the latched publisher alive throughout planning.


class GazeboCones:
    def __init__(self):
        import rospy
        from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=.2)
        self.world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    def update(self, geometry):
        from camera_scene import cone_model_name, cone_sdf, view_markers
        world = self.world()
        if not world.success:
            raise RuntimeError(world.status_message)
        for name in world.model_names:
            if name.startswith('camera_view_cone_'):
                result = self.delete(name)
                if not result.success:
                    raise RuntimeError(result.status_message)
        for view, marker in zip(geometry['views'], view_markers(geometry).markers):
            result = self.spawn(cone_model_name(view), cone_sdf(view), '', marker.pose, 'world')
            if not result.success:
                raise RuntimeError(result.status_message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--configure-rviz', metavar='PATH')
    args, _ = parser.parse_known_args()  # roslaunch also supplies ROS remapping arguments.
    if args.configure_rviz:
        configure_rviz(args.configure_rviz)
        return

    import os
    import rospy
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    from camera_scene import clear_markers, view_markers

    rospy.init_node('camera_view_scene')
    if os.environ.get('SESSION_MODE') != 'sim' or not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Camera cone visualization is available only in simulation')
    # Start clear; the camera tour publishes its targets when requested.
    # Also remove any old cones if this node restarts in an existing scene.
    pending = [json.dumps({'views': []})]

    def receive(message):
        try:
            geometry = json.loads(message.data)
            view_markers(geometry)  # Validate before replacing the displayed targets.
        except (ValueError, KeyError, TypeError, IndexError) as error:
            rospy.logwarn('Ignoring invalid camera geometry: %s', error)
            return
        pending[0] = message.data

    publisher = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1, latch=True)
    subscriber = rospy.Subscriber(GEOMETRY_TOPIC, String, receive, queue_size=1)
    shown, spawned, gazebo = None, None, None
    while not rospy.is_shutdown():
        current = pending[0]
        geometry = json.loads(current)
        if current != shown:
            markers = clear_markers()
            markers.markers.extend(view_markers(geometry).markers)
            publisher.publish(markers)
            shown = current
        if current != spawned:
            try:
                gazebo = gazebo or GazeboCones()
                gazebo.update(geometry)
                spawned = current
            except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
                gazebo = None
                rospy.logwarn_throttle(10, 'Waiting to display camera cones in Gazebo: %s', error)
        # Wall time also keeps markers responsive when simulation is paused.
        rospy.rostime.wallsleep(.5)


if __name__ == '__main__':
    main()
