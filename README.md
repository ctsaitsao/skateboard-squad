# Skateboarding Robot Project
Senior capstone project for ME 495: Robot Design Studio, Northwestern University, Winter and Spring 2020.

## Overview
This is a repository of the code that was used to program the Tiva microcontroller and simulate the robot tricks for the Skateboarding Team of Robot Design Studio 2020. 

## Background
For the academic year of 2019-2020, RDS projects were themed around olympic sports, and this project was inspired by the new olympic sport of skateboarding. 

The goal of this project was to build a skateboarding robot that could accomplish three tricks of choice. The design was constrained in two ways: 1) the robot could only be actuated with two degrees of freedom and 2) the skateboard wheels could not be actuated. 

**Due to the pandemic, the project had to pivot from a physical robot to a simulated one halfway during the course.**

The three tricks that the robot performs are:
1. Dropping-in: a trick with which a skateboarder starts skating a half-pipe by dropping into it from the coping instead of starting from the bottom. For the robot, this meant moving the horizontal component of the COM from outside the half-pipe to past the coping.

   <img src="media/person_drop_in.gif" width="500">

2. Pumping: a technique used to accelerate without the rider's feet leaving the board. For the robot, this meant rapidly moving the vertical component of the COM down when accelerating down the half-pipe and up when decelerating up it. This was definitely the hardest trick to perform, as it required the robot to have considerable top mass and motor power to achieve this high acceleration.

   <img src="media/person_pumping.gif" width="500">

3. The "manual": essentially is a skateboarding wheelie. For the robot, this meant controlling the horizontal component of the COM to be directly above the hind wheel of the skateboard. We used LQR control.

   <img src="media/person_manual.gif" width="500">

## Mechanical Design
Much of Winter 2020 was spent designing the mechanical components of the robot. The overall design resembled an inverted double pendulum, and provided a superior ability to control the robot's center of mass when compared to designs that used linear or decoupled rotational actuators. Most of the robot's weight was concentrated at the top (the pendulum bob) in order to provide the necessary momentum to perform the tricks when manipulated. Fittingly, the robot was nicknamed "Thora" due to its likeness to Thor's hammer.

<img src="media/robot_irl.jpg" height="400"> <img src="media/robot_cad.jpg" height="400">

## Electronics


## Simulation and Trick Demos
Even though the real-life robot could not be used after the pandemic hit, its mechanical properties were used to model a dynamic system in MATLAB and simulate the tricks. See the [simulation README](simulation/README.md) for a more in-depth look at the code.
1. Dropping-in:
   <br>
   <img src="media/drop_in.gif" width="450">
2. Pumping:
   <br>
   <img src="media/pumping.gif" width="450">
3. Manual:
   <br>
   <img src="media/manual.gif" width="450">
   
## Usage
For Tiva code, each `Makefile` must be edited to fit a user's computer. See [Tiva make README](hello_world/README.md) for instructions on how to compile code.

All MATLAB codes have options for all three three tricks but will only run the specified one correctly (correct gains, will plot for the specified trick).

See instructions on how to run simulations in the [simulation README](simulation/README.md).

## Folder Descriptions
- `accelerometer`: Pololu LSM303D accelerometer data retrieval.
- `bno_drivers`: Bosch Sensortec BNO055 sensor driver library.
- `encoder`: communication between CUI AMT11 encoder and Tiva.
- `hello_world`: tests Tiva communication. Includes the [Tiva make README](hello_world/README.md).
- `imu_sparkfun`: streams data from SparkFun IMU (LSM9DS1).
- `pid_controller_on_tiva`: simulating PID control of a rotary mass-spring-damper system using Simulink and Tiva microcontroller.
- `simulation`: MATLAB code that runs a simulations of the skateboard robot. Includes the [simulation README](simulation/README.md)
- `tiva_roboteq`: code that interfaces between the Tiva microcontroller and Roboteq controller for position commands.
