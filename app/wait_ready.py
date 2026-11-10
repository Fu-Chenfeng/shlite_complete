"""Bounded, read-only startup probes for the desktop control panel."""
import argparse
import time

import rospy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('hand', 'ur5', 'sim', 'real'))
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('shlite_startup_probe', anonymous=True)
    deadline = time.monotonic() + args.timeout

    def remaining():
        value = deadline - time.monotonic()
        if value <= 0:
            raise RuntimeError('Timed out waiting for ' + args.phase)
        return value

    if args.phase in ('hand', 'ur5'):
        service = ('/controller_manager/list_controllers' if args.phase == 'hand'
                   else '/ur5_driver/controller_manager/list_controllers')
        rospy.wait_for_service(service, timeout=remaining())
        rospy.ServiceProxy(service, ListControllers)()
    else:
        rospy.wait_for_service('/get_planning_scene', timeout=remaining())
        rospy.wait_for_message('/joint_states', JointState, timeout=remaining())
        if args.phase == 'real':
            service = '/ur5_driver/controller_manager/list_controllers'
            rospy.wait_for_service(service, timeout=remaining())
            while True:
                controllers = rospy.ServiceProxy(service, ListControllers)().controller
                if any(c.name == 'scaled_pos_joint_traj_controller' and c.state == 'running'
                       for c in controllers):
                    break
                remaining()
                rospy.sleep(0.5)
            topic = '/ur5_driver/ur_hardware_interface/robot_program_running'
            while not rospy.wait_for_message(topic, Bool, timeout=remaining()).data:
                remaining()
    print(args.phase + ' is ready.', flush=True)


if __name__ == '__main__':
    main()
