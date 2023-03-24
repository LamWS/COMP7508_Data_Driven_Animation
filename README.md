# COMP7508 Data-Driven Computer Animation

Welcome to the COMP7508 in year 2023!

Here is the code repository of HKU course COMP7508. Any commit and issue will be welcome.

Instructor: [Prof. Taku Komura](https://www.cs.hku.hk/index.php/people/academic-staff/taku)

TAs: @[Mingyi Shi](https://rubbly.cn)  @[Huancheng Lin](https://github.com/LamWS)

![cover](https://user-images.githubusercontent.com/43705353/216566280-3931d201-2814-4e2b-bb65-c64143d9215c.png)

## Instruction

* Always keep the latest version of this repo

  ```
  git clone https://github.com/LamWS/COMP7508_Data_Driven_Animation.git
  ```
* Don't hesitate to seek helps with issue workspace

## Assignment 1 - Basic Character Animation

In this assignment, you will learn the basic data structure and animation creation pipeline and are required to create an animation clip with provided infrastructure. Also, you need to understand the mathematics in FK and IK to read the motion capture files and play with them.

Details: [[subfolder](./assignment_1)]

#### Assessments

- A rendered video with character animation (Task 1, 40%)
- Core code implementation of Forward Kinematics (Task 2, 50%)
- Core code implementation of Inverse Kinematics - CCD IK (Task 3, Bonus)
- Report (10%)

## Assignment 2 - Animation Processing and Scripting

This assignment will teach you how to play with animation data by different algorithms like the interpolation, DTW (Dynamic Time Warping) and a Motion Matching.

Detials: [[subfolder](./assignment_2)]

#### Assessments

* part1_key_framing (30%)
  * Linear interpolation (10%); Slerp Interpolation (15%)
  * Report the different performance by giving different numbers (5%)
* part2_concatenation (40%)
  * Define the search window (10%) + Calculate the sim_matrix (10%);
  * Find the real_i and real_j (10%);
  * The shifting on the root joint position (10%)
* part3_motion_matching (15%)
  * Variable terms (22% - your_variable_num)
* Report (10%) + 2 videos (5%)
  * Including necessary experiment results by *different parameters* (5%) and your *thinking*(5%) for how to produce high quality motions.

## Assignment 3 - TBA

Pls, contact myshi@cs.hku.hk or lamws@connect.hku.hk if there is any question.
