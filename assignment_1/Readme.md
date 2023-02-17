# Assignment 1 - Basic Character Animation

<!-- ## Highlights

* Submission DDL: Feb. 28th -->

## Introduction

In this assignment, you will learn the basic data structure and animation creation pipeline and are required to create an animation clip with provided infrastructure. Also, you need to understand the mathematics in FK and IK to read the motion capture files and play with them.

The experiment environment is re-designed by [GAMES105](https://github.com/GAMES-105/GAMES-105), thanks for the open sourcing.

#### Submission

File format: A compress file [uid_name_assignment1.zip] with:

1. video_file
2. task2_forward_kinematic.py, task3_inverse_kinematic.py
3. uid_name_1.pdf

## Examples

<p align="center">
Desired results for Task 1 (from students in 2022)
</p>

![video3](https://user-images.githubusercontent.com/7709951/158897304-7759b671-0a62-4c64-934c-d6be46fdbca1.gif)

<p align="center">
Desired results for Task 2 & 3
</p>

![20230214001730](https://user-images.githubusercontent.com/7709951/218512528-a44a8ffc-e9bb-43e5-8b6a-ebbdfd1e8141.jpg)


## Task 1 - A rendered video with character animation(40%)

- Download [Blender](https://www.blender.org/download/)
- Import the [provided mesh](https://github.com/Shimingyi/COMP3360_Data_Driven_Animation/blob/2023/assignment_1/task1/hm.obj) (feel free to use your mesh if you like)
- Define your key joints and skeleton
- Rigging/Skinning
- Design keyframes animation (feel free to make use of your creativity to add any objects you like)
- Render the video (make use of lights, camera)

## Before Task 2 & 3

#### Enviroment Setting

Task 2 & 3 requires Python 3 runtime, and Panda3d will be used for rendering.

```shell
# recommend to use Anaconda to manage enviroment 
conda create -n comp3360 python=3.8
conda activate comp3360
conda install numpy scipy
pip install panda3d

# Enviromtns verification. After running, you should see a skeleton in a 3D space
# If you are using Appli-Silicon Chip and meet black screen problem, check this ISSUE: https://github.com/Shimingyi/COMP3360_Data_Driven_Animation/issues/4
cd ./assignment_1
python env_test.py
```

#### Pre-knowledge

Human Skeleton and Motion

* As introduced in the tutorial, a body skeleton is a collection of bones representing the connection of body joints. The body joint is a 3D point in space, with (x, y, z) positional information as primary data. Sometimes, the offset between two joints will also be used, formulated by (x2-x1, y2-y1, z2-z1) with a vector.
* When we rotate a bone, a rotation value will be applied to the parent joint of this bone. For example, if we have joint_1 in (0, 0) and joint_2 in (1, 0), then a 45-degree anticlockwise rotation on joint_1 will yield the positional movement of joint_2, from (1, 0) to (sin45, sin45).
* Global/Local Rotation. The local rotation always means the relative rotation from the parent to the child's joint, and the global rotation (orientation) represents the global rotation of the whole 3D space. In the above case, after the rotation, the orientation of J1 and J2 is both 45 degrees, but the local rotation of J2 keeps the same as 0 degrees, only the local rotation of J1 will be updated to 45 degrees.
* BVH file is a combination of joint offset and local rotations, and you can find more details with this [link](https://research.cs.wisc.edu/graphics/Courses/cs-838-1999/Jeff/BVH.html).
* Quaternion is a rotation representation, you can find the details and its history with [wiki](https://en.wikipedia.org/wiki/Quaternion). In this project, (x, y, z, w) is the default order.

Vector and Rotation

* NumPy and SciPy will be used.
* Vector, Quaternion, Rotation_by_Vector, Euler Rotation, and Matrix defination:
  ```python
  vector = np.array(1, 1, 1)
  quat = R.from_quat([ 0.        ,  0.80976237,  0.53984158, -0.22990426])
  rot_vec = R.from_rotvec([0, 3, 2])
  euler_angle = R.from_euler('XYZ', [-109.60643629,  -21.85973481, -164.48716608], degrees=True)
  matrix = R.from_matrix([[-0.89428806,  0.24822376, -0.37233564],
         [-0.24822376,  0.41714214,  0.8742868 ],
         [ 0.37233564,  0.8742868 , -0.31143019]])
  ```
* Rotate an vector
  ```python
  orentation = # R.from_quat / R.from_rotvec / R.from_matrix 
  orientation.apply(vector)
  ```
* More related details please follow: [https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html ](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html)

## Task 2 - Forward Kinematics

In this task, we will load the data from the BVH file and visualize the skeleton based on two data types - Offset and Rotation. For convenience, almost code modules (file parse, visualization) have been given, and only core implementation needs to be filled.

You are required to implement two functions in *[task2_forward_kinematic.py](https://github.com/Shimingyi/COMP3360_Data_Driven_Animation/blob/2023/assignment_1/task2_forward_kinematic.py "task2_forward_kinematic.py").*

1. Starting from Line 8, there is a function called *part1_show_T_pose*. We should fill in the code based on the instruction. Then uncomment line 131 and run the script for calling the function. You can see a T-Pose centered with (0, 0, 0).
2. Starting from Line 45, there is a function called *part2_forward_kinametic*. We should fill in the code based on the instruction. Then uncomment line 134 and run the script for calling the function. You can see a walking motion if show_animation is set to True otherwise a static 

Screenshot of T-pose and walking motion will be expected in the assignment report.

## Task 3 - Inverse Kinematics - CCD IK

In Task 1, you might have been familiar with controlling the whole body pose by a few control points, but it's a built-in blender function. And in this task, you will try to implement a simple IK algorithm yourself!

The CCD IK is the default option in the code, we provided almost code modules already, and you only need to fill in the core algorithm as shown here:

![20230214113236](https://user-images.githubusercontent.com/7709951/218632375-a388278f-b185-405c-bf65-dd44d7459ea6.jpg)

1. Given P1, ... P4, and target position of P4
2. In each iteration:
   1. Find a current cursor, for example -> P3
   2. Find the vector from P3 to P4
   3. Find the vector from P3 to target
   4. Rotate the vector P3->P4 to the direction of P3->target (should be careful!)
   5. Update chain orentation
   6. move the cursor to P2

You are required to implement one function in *[task3_inverse_kinematic.py](task3_inverse_kinematic.py)* and try different IK settings(iteration number/start joint/end joint/target position), then report them with a screenshot and comparison table.

* The function is started from line 48. And only a few lines will be needed to fill.
* Any other IK implementation will be welcome, and a bonus will be given.

## Task 4 - Report

* PDF format, no page size requriment so you can also prepare it with powerpoint or keynote
* The first two lines should introduce your NAME and UID.
* Screenshot for each subtask need to be included in the report.
