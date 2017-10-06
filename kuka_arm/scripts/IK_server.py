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
import math
import numpy as np
import os


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
        wc_x, wc_y, wc_z, sym_B = symbols('wc_x, wc_y, wc_z, sym_B')

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

        # Create transformation matrices w.r.t. base_link frame
        T0_2 = T0_1 * T1_2
        T0_3 = T0_2 * T2_3
        T0_4 = T0_3 * T3_4
        T0_5 = T0_4 * T4_5
        T0_6 = T0_5 * T5_6
        T0_G = T0_6 * T6_G

        # Lambda functions for fast numerical evaluation in the for loop
        eval_T0_3 = lambdify((q1, q2, q3), T0_3)
        eval_T0_G = lambdify((q1, q2, q3, q4, q5, q6), T0_G)

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
        Rc_z = Matrix([[-1, 0, 0, 0],
                       [ 0,-1, 0, 0],
                       [ 0, 0, 1, 0],
                       [ 0, 0, 0, 1]])

        # Rotate about y-axis -90 degrees
        Rc_y = Matrix([[ 0, 0,-1, 0],
                       [ 0, 1, 0, 0],
                       [ 1, 0, 0, 0],
                       [ 0, 0, 0, 1]])

        # Combined corrective rotation matrix
        R_corr = Rc_z * Rc_y

        # Homogeneous transform for given end-effector poses
        R_rpy = (R_z * R_y * R_x * R_corr)

        # Lambda function for fast numerical evaluation in the for loop
        eval_R_rpy = lambdify((r, p, y), R_rpy)

        # Lambda functions and variables for fast numerical evaluation
        A = math.sqrt(s[a3]**2 + s[d4]**2)
        C = s[a2]
        eval_B = lambdify((wc_x, wc_y, wc_z),
                          sqrt((sqrt(wc_x**2 + wc_y**2) - s[a1])**2 +
                          (wc_z - s[d1])**2))
        eval_theta2 = lambdify((wc_x, wc_y, wc_z, sym_B), (pi/2 - acos((sym_B**2 +
                                C**2 - A**2)/(2*sym_B*C)) -
                                atan2(wc_z - s[d1], sqrt(wc_x**2+wc_y**2) - s[a1])))
        eval_theta3 = lambdify(sym_B, (pi - acos((A**2 + C**2 - sym_B**2)/(2*A*C)) -
                                      (pi/2 - atan2(s[a3], s[d4]))))

        ###

        # Initialize service response
        ee_error_list = []
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position and orientation from request
	        # px, py, pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### My IK code here
	        # Numerically evaluate rotation matrix of end-effector orientation
            R_rpy_num = eval_R_rpy(roll, pitch, yaw)

	        # Calculate joint angles using Geometric IK method
            # Wrist center position
            grip_length = s[d7]
            wx = px - grip_length * R_rpy_num[0, 2]
            wy = py - grip_length * R_rpy_num[1, 2]
            wz = pz - grip_length * R_rpy_num[2, 2]

            # Solve numerical inverse position using law of cosines
            B = eval_B(wx, wy, wz)
            theta1 = math.atan2(wy, wx)
            theta2 = eval_theta2(wx, wy, wz, B)
            theta3 = eval_theta3(B)

            # Update numerical transformation matrix using first three angles
            T0_3_num = eval_T0_3(theta1, (theta2 - np.pi/2), theta3)

            # Solve numerical inverse orientation
            R_rhs = np.matmul(T0_3_num.T, R_rpy_num)

            theta4 = math.atan2(R_rhs[2,2], -R_rhs[0,2])
            theta5 = math.atan2(math.sqrt(R_rhs[2,2]**2 + R_rhs[0,2]**2), R_rhs[1,2])
            theta6 = math.atan2(-R_rhs[1,1], R_rhs[1,0])

            # Update numerical total transform with all joint angles
            T0_G_num = eval_T0_G(theta1, theta2 - np.pi/2, theta3, theta4, theta5, theta6)

            # Find EE error using FK
            ee_x_e = abs(T0_G_num[0, 3] - px)
            ee_y_e = abs(T0_G_num[1, 3] - py)
            ee_z_e = abs(T0_G_num[2, 3] - pz)
            ee_offset = math.sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)

            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)
            ee_error_list.append(ee_offset)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        plot_ee_error(ee_error_list) # Call error plotting function
        return CalculateIKResponse(joint_trajectory_list)


def plot_ee_error(error_list):
    # plots error using matplotlib
    global count
    plt.figure()
    plt.plot(error_list)
    plt.xlabel('Requested Pose #')
    plt.ylabel('Total error')
    fname = "../../misc_images/error_plots/error_curve" + str(count)
    plt.savefig(fname)
    plt.close()
    count += 1

def IK_server():
    # Variable for naming error plots
    global count
    count = 0
    # Remove old error plots
    filelist = os.listdir("../../misc_images/error_plots")
    for f in filelist:
        os.remove("../../misc_images/error_plots/" + f)
    # Initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
