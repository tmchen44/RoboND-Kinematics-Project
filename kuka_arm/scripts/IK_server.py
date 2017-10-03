#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
import matplotlib.pyplot as plt


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### My FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # Theta
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        q, d, a, alpha = symbols('q, d, a, alpha')
        r, p, y = symbols('r, p, y')

        # Create Modified DH parameters
        s = {alpha0:     0, a0:      0, d1:  0.75,
             alpha1: -pi/2, a1:   0.35, d2:     0,
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:  1.50,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303, q7: 0}

        # Define Modified DH Transformation matrix
        T = Matrix([[           cos(q),           -sin(q),           0,             a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                0,                 0,           0,             1]])

        # Create individual transformation matrices
        T0_1 = T.subs([(alpha, alpha0), (a, a0), (d, d1), (q, q1)])
        T0_1 = T0_1.subs(s)
        T1_2 = T.subs([(alpha, alpha1), (a, a1), (d, d2), (q, q2)])
        T1_2 = T1_2.subs(s)
        T2_3 = T.subs([(alpha, alpha2), (a, a2), (d, d3), (q, q3)])
        T2_3 = T2_3.subs(s)
        T3_4 = T.subs([(alpha, alpha3), (a, a3), (d, d4), (q, q4)])
        T3_4 = T3_4.subs(s)
        T4_5 = T.subs([(alpha, alpha4), (a, a4), (d, d5), (q, q5)])
        T4_5 = T4_5.subs(s)
        T5_6 = T.subs([(alpha, alpha5), (a, a5), (d, d6), (q, q6)])
        T5_6 = T5_6.subs(s)
        T6_G = T.subs([(alpha, alpha6), (a, a6), (d, d7), (q, q7)])
        T6_G = T6_G.subs(s)

        T0_2 = T0_1 * T1_2
        T0_3 = T0_2 * T2_3
        T0_4 = T0_3 * T3_4
        T0_5 = T0_4 * T4_5
        T0_6 = T0_5 * T5_6
        T0_G = T0_6 * T6_G

        # Elementary rotations about principal axes
        R_x = Matrix([[       1,       0,       0,       0],
                      [       0,  cos(r), -sin(r),       0],
                      [       0,  sin(r),  cos(r),       0],
                      [       0,       0,       0,       1]])

        R_y = Matrix([[  cos(p),       0,  sin(p),       0],
                      [       0,       1,       0,       0],
                      [ -sin(p),       0,  cos(p),       0],
                      [       0,       0,       0,       1]])

        R_z = Matrix([[  cos(y), -sin(y),       0,       0],
                      [  sin(y),  cos(y),       0,       0],
                      [       0,       0,       1,       0],
                      [       0,       0,       0,       1]])

        # Corrective rotation matrix
        # Rotate about z-axis 180 degrees
        Rc_z = Matrix([[ cos(pi), -sin(pi),       0,       0],
                       [ sin(pi),  cos(pi),       0,       0],
                       [       0,        0,       1,       0],
                       [       0,        0,       0,       1]])

        # Rotate about y-axis -90 degrees
        Rc_y = Matrix([[ cos(-pi/2),       0, sin(-pi/2),       0],
                       [          0,       1,          0,       0],
                       [-sin(-pi/2),       0, cos(-pi/2),       0],
                       [          0,       0,          0,       1]])

        # Corrective homogeneous transform
        R_corr = Rc_z * Rc_y

        # Homogeneous transform for given poses
        R_rpy = (R_z * R_y * R_x * R_corr)
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

            ### My IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_rpy_num = R_rpy.evalf(subs={r: roll, p: pitch, y: yaw})

	        # Calculate joint angles using Geometric IK method
            # Wrist center position
            grip_length = s[d7]
            wx = px - grip_length * R_rpy_num[0, 2]
            wy = py - grip_length * R_rpy_num[1, 2]
            wz = pz - grip_length * R_rpy_num[2, 2]

            # Solve inverse position using law of cosines
            A = sqrt(a3**2 + d4**2)
            B = sqrt((sqrt(wx**2 + wy**2) - a1)**2 + (wz - d1)**2)
            C = a2

            theta1 = atan2(wy, wx)
            theta2 = (pi/2 - acos((B**2+C**2-A**2)/(2*B*C)) -
                      atan2(wz - d1, sqrt(wx**2+wy**2) - a1)).evalf(subs=s)
            theta3 = (pi - acos((A**2+C**2-B**2)/(2*A*C)) -
                      (pi/2 - atan2(a3, d4))).evalf(subs=s)

            # Update dictionary with first 3 calculated theta values
            s.update({q1: theta1, q2: theta2 - pi/2, q3: theta3})

            # Update transformation matrices using first three angles
            T0_3_num = T0_3.evalf(subs=s)

            # Solve inverse orientation
            R_rhs = T0_3_num.T * R_rpy_num
            theta4 = atan2(R_rhs[2,2], -R_rhs[0,2])
            theta5 = atan2(sqrt(R_rhs[2,2]**2 + R_rhs[0,2]**2), R_rhs[1,2])
            theta6 = atan2(-R_rhs[1,1], R_rhs[1,0])

            # Update dictionary with last 3 calculated theta values
            s.update({q4: theta4, q5: theta5, q6: theta6})

            # Update total transform with all joint angles
            T0_G_num = T0_G.evalf(subs=s)

            # Find FK EE error
            ee_x_e = abs(T0_G_num[0, 3] - px)
            ee_y_e = abs(T0_G_num[1, 3] - py)
            ee_z_e = abs(T0_G_num[2, 3] - pz)
            ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)

            # Plot error using matplotlib
            # plt.figure(1)
            # plt.plot(ee_offset)
            # plt.xlabel('Pose request')
            # plt.ylabel('Error (in meters)')
            # plt.show()

            # Print error amounts
            print "total offset is: ", ee_offset
            print "x error is: ", ee_x_e
            print "y_error is: ", ee_y_e
            print "z_error is: ", ee_z_e
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
