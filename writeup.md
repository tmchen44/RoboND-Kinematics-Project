# Project: Kinematics Pick & Place
---

**Steps to complete the project:**

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/01-dh_table.png
[image2]: ./misc_images/02-joint_transforms.png
[image3]: ./misc_images/03-base_gripper_transform.png
[image4]: ./misc_images/04-inverse_position.png
[image5]: ./misc_images/05-inverse_orientation.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
#### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To derive the DH parameters:
1. I start by drawing the robotic arm in the configuration where all joint angles are equal to zero. See **Figure 1** below for details.
2. I start drawing in the Z axes where the joints are, followed by the X axes. These axes with their corresponding origin points are shown in red.
3. I calculate the DH parameters using the given URDF file. Since the joint axes defined in the URDF file are different from the joint axes defined here, I used the diagram shown in the lesson to help determine how to relate the joint origins in the URDF file to the joint axes shown here. The parameters are calculated accordingly. See the bottom of **Figure 1** for the DH parameter table.

**Figure 1 - Robot Arm Drawing with DH Parameter Table**

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector (gripper) pose.

Please see below for the individual transformation matrices about each joint, as well as the homogeneous transform between base_link and gripper_link. The derivation of each matrix is also explained.

Page 1
![alt text][image2]

Page 2
![alt text][image3]

#### 3. Decouple the Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics; doing so, derive the equations to calculate all individual joint angles.

Please see below for the derivation of equations for each joint angle. Page 1 shows Inverse Position Kinematics, and page 2 shows Inverse Orientation Kinematics.

Page 1
![alt text][image4]

Page 2
![alt text][image5]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

In the upper half of `IK_server.py`, I use Sympy to create the homogeneous transform matrices for transforming between each joint, as well as the homogeneous transform matrices relating each link back to the base_link (lines 57-79). Since all joints are revolute joints, only q (theta) varies over time, and all other DH parameters are substituted into the transform matrices as numerical quantities. Furthermore, for any matrices or equations that are used in the IK portion of the code (the for loop), I use Sympy's lambdify function to turn these expressions into functions that can be numerically evaluated quickly via Numpy (lines 82-83). Using Sympy, I also create the end-effector roll-pitch-yaw transform for relating the given end-effector pose to the base_link (lines 85-121). At the end of the section, I create some of the expressions necessary to solve the inverse position problem and lambdify those expressions for faster numerical evaluation in the for loop (lines 123-133).

In the IK portion of `IK_server.py` (the for loop), I start by numerically evaluating the end-effector roll-pitch-yaw transform (line 157). I use this to solve for the wrist center position (lines 159-164). Then, using the wrist center position, I solve the inverse position problem using trigonometry and the expressions that I "lambdified" in the first half of the code (lines 166-170). Having solved the first three joint angles, I input these into the transformation matrix relating the third joint to base_link (`T0_3_num`, line 173). I then use this to solve for the last three joint angles (lines 172-180). After solving for all six joint angles, I then substitute the six joint angles back into the homogeneous transformation matrix relating the gripper_link back to the base_link and use the results to calculate the error in end-effector position (lines 182-189). I then plot the error for each of the poses received from the request before returning the joint angles in the IK response (lines 204-214). The error plots can be found as `.png` files in this directory: `RoboND-Kinematics-Project/misc_images/error_plots`.

The results produced by `IK_server.py` are satisfactory. The magnitude of calculated error for each end-effector position is very small, on the order of 10^-15. If I were to work on this project further, I would give more consideration to the calculated joint angles and how they change between different poses. At certain times, it seemed that the robotic arm would undergo unnecessary rotations to reach the next pose. Furthermore, although the IK portion of the code (the for loop) performs quickly due to the numerical "lambdification" of the Sympy expressions, the FK portion (outside of the for loop) is still slow, due to several symbolic substitutions and symbolic matrix multiplications. For a further performance increase, I would consider not using Sympy at all in `IK_server.py` and modify the code to just utilize numpy and the math module for solving FK and IK. Although Sympy is a very useful tool for mathematical analysis, mathematical symbolic manipulation in general seems to be a slow process.
