from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##
    ### IK Code
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
         alpha6:     0, a6:      0, d7: 0.303}

    # Define Modified DH Transformation matrix
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])

    # Elementary rotations about principal axes
    R_x = Matrix([[       1,       0,       0],
                  [       0,  cos(r), -sin(r)],
                  [       0,  sin(r),  cos(r)]])

    R_y = Matrix([[  cos(p),       0,  sin(p)],
                  [       0,       1,       0],
                  [ -sin(p),       0,  cos(p)]])

    R_z = Matrix([[  cos(y), -sin(y),       0],
                  [  sin(y),  cos(y),       0],
                  [       0,       0,       1]])

    # Corrective rotation matrix
    # Rotate about z-axis 180 degrees
    Rc_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                   [sin(pi),  cos(pi), 0, 0],
                   [      0,        0, 1, 0],
                   [      0,        0, 0, 1]])

    # Rotate about y-axis -90 degrees
    Rc_y = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                   [          0, 1,          0, 0],
                   [-sin(-pi/2), 0, cos(-pi/2), 0],
                   [          0, 0,          0, 1]])

    # Corrective homogeneous transform
    T_corr = Rc_z * Rc_y
    R_corr = T_corr[0:3, 0:3]

    px = test_case[0][0][0]
    py = test_case[0][0][1]
    pz = test_case[0][0][2]

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [test_case[0][1][0], test_case[0][1][1],
            test_case[0][1][2], test_case[0][1][3]])

    # Compensate for rotation discrepancy between DH parameters and Gazebo
    R_rpy = (R_z * R_y * R_x * R_corr).evalf(subs={r: roll, p: pitch, y: yaw})

    # Calculate joint angles using Geometric IK method
    # Wrist center position
    grip_length = s[d7]
    wx = px - grip_length * R_rpy[0, 2]
    wy = py - grip_length * R_rpy[1, 2]
    wz = pz - grip_length * R_rpy[2, 2]

    # Inverse position using law of cosines
    theta1 = atan2(wy, wx)
    A = sqrt(a3**2 + d4**2)
    B = sqrt((sqrt(wx**2 + wy**2) - a1)**2 + (wz - d1)**2)
    C = a2
    theta2 = (pi/2 - acos((B**2+C**2-A**2)/(2*B*C)) -
              atan2(wz - d1, sqrt(wx**2+wy**2) - a1)).evalf(subs=s)
    theta3 = (pi - acos((A**2+C**2-B**2)/(2*A*C)) -
              (pi/2 - atan2(a3, d4))).evalf(subs=s)

    s.update({q1: theta1, q2: theta2-pi/2, q3: theta3})

    # Create individual transformation matrices based on first three angles
    T0_1 = T.subs([(alpha, alpha0), (a, a0), (d, d1), (q, q1)])
    T0_1 = T0_1.subs(s)
    T1_2 = T.subs([(alpha, alpha1), (a, a1), (d, d2), (q, q2)])
    T1_2 = T1_2.subs(s)
    T2_3 = T.subs([(alpha, alpha2), (a, a2), (d, d3), (q, q3)])
    T2_3 = T2_3.subs(s)

    T0_2 = T0_1 * T1_2
    T0_3 = T0_2 * T2_3
    R0_3 = T0_3[0:3, 0:3]

    # Inverse orientation
    R_rhs = R0_3.T * R_rpy
    theta4 = atan2(R_rhs[2,2], - R_rhs[0,2])
    theta5 = atan2(sqrt(R_rhs[2,2]**2 + R_rhs[0,2]**2), R_rhs[1,2])
    theta6 = atan2(-R_rhs[1,1], R_rhs[1,0])

    # Update dictionary with calculated theta values
    s.update({q4: theta4, q5: theta5, q6: theta6, q7: 0})
    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    # Continue to create individual transformation matrices
    T3_4 = T.subs([(alpha, alpha3), (a, a3), (d, d4), (q, q4)])
    T3_4 = T3_4.subs(s)
    T4_5 = T.subs([(alpha, alpha4), (a, a4), (d, d5), (q, q5)])
    T4_5 = T4_5.subs(s)
    T5_6 = T.subs([(alpha, alpha5), (a, a5), (d, d6), (q, q6)])
    T5_6 = T5_6.subs(s)
    T6_G = T.subs([(alpha, alpha6), (a, a6), (d, d7), (q, q7)])
    T6_G = T6_G.subs(s)

    T0_4 = T0_3 * T3_4
    T0_5 = T0_4 * T4_5
    T0_6 = T0_5 * T5_6
    T0_G = T0_6 * T6_G
    T3_6 = T3_4 * T4_5 * T5_6
    T_total = T0_G * T_corr

    # Extract rotation matrices from the transformation matrices
    R0_1 = T0_1[0:3, 0:3]
    R1_2 = T1_2[0:3, 0:3]
    R2_3 = T2_3[0:3, 0:3]
    R3_4 = T3_4[0:3, 0:3]
    R4_5 = T4_5[0:3, 0:3]
    R5_6 = T5_6[0:3, 0:3]
    R6_G = T6_G[0:3, 0:3]
    R0_2 = T0_2[0:3, 0:3]
    R0_3 = T0_3[0:3, 0:3]
    R0_4 = T0_4[0:3, 0:3]
    R0_5 = T0_5[0:3, 0:3]
    R0_6 = T0_6[0:3, 0:3]
    R0_G = T0_G[0:3, 0:3]
    R3_6 = T3_6[0:3, 0:3]
    R_total = T_total[0:3, 0:3]

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [T0_5[0, 3], T0_5[1, 3], T0_5[2, 3]] # <--- Load your calculated WC values in this array
    your_ee = [T0_G[0, 3], T0_G[1, 3], T0_G[2, 3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
