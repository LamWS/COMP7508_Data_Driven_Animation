# Assignment 3

## Highlights

### Submission Due:

- Apr. 15th, 23:59

### Submission

File format: A compressed file **[uid_name_assignment3.zip]** with:

1. your `rigid_body_dynamic.py`
2. a recorded video of your simulator
3. your report `uid_name_3.pdf`
4. your own mesh.obj (if any)

## Introduction

Rigid body simulation is a powerful tool used in a variety of fields, from robotics and engineering to computer graphics and animation. It involves modeling and simulating the motion of solid objects that maintain their shape and size, even when subjected to external forces and torques.

In this assignment, we will explore how to implement a simple rigid body simulation using Python and Taichi. Taichi is a high-performance programming language designed for numerical computing and machine learning, and it provides a user-friendly interface for writing efficient simulations that can run on both CPUs and GPUs.

We will begin by reviewing the basic principles of rigid body dynamics, including the equations of motion and the kinematics of rotation and translation.

Using these tools, we will develop a Python program that simulates the motion of a rigid body in response to external forces and torques, such as gravity, friction, and collisions. 

By the end of this assignment, you will have gained a solid understanding of the principles of rigid body simulation and how to implement them using Python and Taichi. You will also have developed practical skills in numerical computing, programming, and visualization that can be applied to a wide range of scientific and engineering problems.

## Examples

Here are some examples of the sucessfully implemented simulator.

Ball Dropping without changing any of the simulation parameters.

https://user-images.githubusercontent.com/43705353/231971484-f043cb9c-b53b-43cc-9220-c69b3c60fc4c.mov


Ball Dropping with initial velocity [3, 0, 0].

https://user-images.githubusercontent.com/43705353/231971536-5b57bb7c-e12c-49bd-806f-c7208e933eb1.mov


Ball Dropping with initial velocity [3, 0, 0] and collision damping stiffness 1e4, friction coefficient 0.5 on the ground.

https://user-images.githubusercontent.com/43705353/231973213-6caa0c81-3eab-4b0f-bbbf-e878ca193215.mov



## Environment

### New environment

```bash
# recommend to use Anaconda to manage enviroment 
$ conda create -n comp7508 python=3.10
$ conda activate comp7508
$ conda install numpy scipy
$ pip install panda3d taichi

$ cd ./assignment_3
$ python rigid_body_dynamic.py
```

### From the existing environment for Assignment1&2

```bash
$ conda activate comp7508
$ pip install taichi
```

## Task 1 - Basic Rigid Body Simulator (90%)

You are required to implement **7** `TODO` to update the rigid body state in rigid_body_dynamic.py. Please check the hints in the comments.

Feel free to use your mesh or change the simulation parameters(initial_velocity, etc.).

You are strongly suggested to spend 30-60 mins to go through https://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf, before doing any implementation.

## Task 2 (Bonus)

Once you have implemented the basic simulator, there is an opportunity to enhance its functionality by adding additional features. For instance, you may consider implementing damping and friction to improve the simulator. 

If you are unable to implement additional features, you may still propose your ideas on how to improve the simulator in your report (For example, One possible way to model friction is to use the equation $\mu N v_d$, where mu represents the friction coefficient, N represents the pressure, and vd represents the direction of velocity.). 

If your proposal is deemed reasonable, you may be rewarded with a bonus.

## Task 3 Report (10%)

- PDF format, no page size requirement, so you can also prepare it with PowerPoint or keynote or markdown
- The first two lines should introduce your NAME and UID.
- Should include 
  - Your simulation parameters. (initial_velocity, initial_angular_velocity, etc.)
  - Task 2, Your idea to improve the simulator. (If any)
