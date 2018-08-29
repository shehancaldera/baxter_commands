#!/usr/bin/env python

"""
	Joint Angle Recorder script by Shehan Caldera(shehancaldera@gmail.com)

Prints and Records Joint Angles for Baxter Robotic Arms

"""

import argparse
import struct
import sys
import time

import rospy
import baxter_interface
from baxter_interface import RobotEnable
import cv2
import cv_bridge

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from sensor_msgs.msg import (
    Image,
)


if __name__ == '__main__':

    rospy.init_node("Record_Baxter_Joint_Angles")
    RobotEnable().enable()

    navigator_arm_r = baxter_interface.Navigator('right')
    navigator_arm_l = baxter_interface.Navigator('left')

    robot_arm_r = baxter_interface.Limb('right')
    robot_arm_l = baxter_interface.Limb('left')

    while True:
    	if navigator_arm_l.button0:
		print robot_arm_l.joint_angles()

    	elif navigator_arm_r.button0:
		print robot_arm_r.joint_angles()

     	elif navigator_arm_l.button1 or navigator_arm_r.button1:
		break

    


    sys.exit()
