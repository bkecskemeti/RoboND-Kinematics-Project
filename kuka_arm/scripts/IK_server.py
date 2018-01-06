#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

from kinematics import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here

        # Create symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1 ,a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Modified DH params
        DH_Table = { alpha0:      0, a0:      0, d1:  0.75, q1:          q1,
                     alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2. + q2,
                     alpha2:     0., a2:   1.25, d3:     0, q3:          q3,
                     alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:          q4,
                     alpha4:  pi/2., a4:      0, d5:     0, q5:          q5,
                     alpha5: -pi/2., a5:      0, d6:     0, q6:          q6,
                     alpha6:      0, a6:      0, d7: 0.303, q7:           0 }

        T0_1 = transformation_matrix(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = transformation_matrix(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = transformation_matrix(alpha2, a2, d3, q3).subs(DH_Table)

        # EE rotation matrix
        r, p, y = symbols('r p y')

        ROT_EE_form = rotation_base_ee(r,p,y)

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            EE = Matrix([[px],[py],[pz]])

            ### Your IK code here
            ROT_EE = ROT_EE_form.subs({'r': roll, 'p': pitch, 'y': yaw})
            WC = EE - (0.303) * ROT_EE[:,2]

            (theta1, theta2, theta3) = inverse_kinematics_1(WC)

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R3_6 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}).inv('LU') * ROT_EE

            (theta4, theta5, theta6) = inverse_kinematics_2(R3_6)

            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
