#!/usr/bin/env python3
"""Read-only diagnostics; never sends a trajectory or switches controllers."""
import os
import rospy
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import Bool

rospy.init_node("shlite_status", anonymous=True)
mode = os.environ.get("SESSION_MODE", "real")
services = [("hand" if mode == "real" else "simulation", "/controller_manager/list_controllers")]
if mode == "real":
    services.append(("UR5", "/ur5_driver/controller_manager/list_controllers"))
for label, service in services:
    try:
        rospy.wait_for_service(service, timeout=3)
        reply = rospy.ServiceProxy(service, ListControllers)()
        for controller in reply.controller:
            print(label, "controller:", controller.name, controller.state)
    except (rospy.ROSException, rospy.ServiceException) as exc:
        print(label, "controller status unavailable:", exc)
if mode == "real":
    try:
        status = rospy.wait_for_message("/ur5_driver/ur_hardware_interface/robot_program_running", Bool, timeout=3)
        print("UR5 External Control program running:", status.data)
    except rospy.ROSException:
        print("No UR5 program-running message received within 3 seconds.")
print("Simulation clock:", rospy.get_param("/use_sim_time", False))
