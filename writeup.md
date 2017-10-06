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
1. I start with drawing the robotic arm in the configuration where all joint angles are equal to zero. See **Figure 1** below for details.
2. I start drawing in the Z axes where the joints are, followed by the X axes. These axes with the origin points are shown in red.
3. I calculate the DH parameters using the given URDF file. The joint axes defined in the URDF file are different from the joint axes defined here, so I used the diagram shown in the lesson to help determine how the joint origins in the URDF file differed from the joint axes shown here. The parameters are calculated accordingly. See the table below or the bottom of **Figure 1** for the DH parameter table.


**Figure 1 - Robot Arm Drawing with DH Parameter Table**

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Please see below for the individual transformation matrices about each point, as well as the homogeneous transform between base_link and gripper_link. The derivation of each matrix is also explained.

Page 1
![alt text][image2]

Page 2
![alt text][image3]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics; doing so, derive the equations to calculate all individual joint angles.

Please see below for the derivation of equations for each joint angle. Page 1 shows the Inverse Position problem, and page 2 shows the Inverse Orientation problem.

Page 1
![alt text][image4]

Page 2
![alt text][image5]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
