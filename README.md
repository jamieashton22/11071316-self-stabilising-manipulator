# 11071316-self-stabilising-manipulator
Repository for MATLAB simulation and embedded firmware developed for the dissertation 
## 'Design and Control of a Self-Stabilising Robotic Manipulator'
submitted 1st May 2026.

The project consists of two main components:
MATLAB simulation scripts used to model the manipulator, the IMU, compare sensor fusion algorithms, and validate the control strategy
The embedded firmware written in C++ using PlatformIO to be used on an Arduino Mega 2560, integrating the control and sensor-fusion.
All simulation and firmware code was developed as part of this project unless otherwise stated below. Firmware code and README.md will be updated until the final demonstration as improvements are made to the system. 

## Repo Structure
The repo consists of two main folders. 'simulation/' is used for the MATLAB modelling files. 'firmware/' is used for the firmware written in C++ using PlatformIO.

## simulation/

- ctrl_sim_setpoint.m : Used to validate the control strategy in tracking a desired setpoint. Kinematic model of the manipulator is set-up in section 'ROBOT' using the Robotics System Toolbox. Desired setpoint (pd) and gains can be changed in section 'Initialisation'. Results are saved as a .mat file called 'ctrl_setpoint_results.mat'. There is also an animation of the robot that can be uncommented for visualisation.
- ctrl_sim_trajectory.m : Used to validate the control strategy in tracking a desired trajectory. Four trajectories can be tracked (circle, helix, square, figure-eight). To select a trajectory, uncomment that trajectories section and comment the others. Kinematic model of the manipulator is set-up in section 'ROBOT' using the Robotics System Toolbox. Gains can be changed in section 'Initialisation'. Results are saved as a .mat file called 'ctrl_traj_results.mat'. There is also an animation of the robot that can be uncommented for visualisation.
- sensor_sim.m : Used to model an IMU (MPU-6050) and implement three sensor fusion algorithms. Complementary filter is set up in the code. Kalman-based filter uses the imufilter object from the Sensor Fusion and Tracking Toolbox.  Madgwick filter uses MATLAB file 'Madgwick.m' adapted from GitHub user cindyli-13. “Madgwick filter simulation (github repository).” Available: https://github.com/cindyli-13/Madgwick-Filter-Simulation.git. All three sensor fusion algorithms are implemented simultaneously. Results are saved as a .mat file called 'sensor_results.mat'. IMU parameters are extracted from the MPU-6050 datasheet.

## Requirements
- MATLAB Version: 24.2.0.2712019 (R2024b) recommended
- Sensor Fusion and Tracking Toolbox
-  Robotics System Toolbox
-  Control System Toolbox
-  Madgwick.m

## Setup
- Clone repository
- Open MATLAB
- Select simulation/ folder from cloned repo as working directory

## firmware/
-robotic_arm_setpoint/ : PlatformIO project folder containing source files and all dependencies to run main.cpp which contains the functionality. Section 'MACROS and GLOBALS' used to set key parameters such as gains and DH parameters. Section 'IMU CLASS' contains definition for class IMU used to obtain raw sensor data and carry out sensor fusion. Section 'ROBOT ARM CLASS' contains definition for class RobotArm and section 'MEMBER FUNCTION' contains definitions for all its methods, this includes key methods relating to the manipulator such as forward kinematics computation, Jacobian computation, DLS pseudoinverse, control step etc. Section 'OBJECTS' contains the object declarations for the IMU, robot arm and Servo Driver. Section 'SETUP' contains the setup. Section 'LOOP' contains the main loop of the program. 

## Requirements
- Visual Studio Code, Ver. 1.106.3
- PlatformIO, Ver. 6.1.19

## Dependencies
- tomstewart89/BasicLinearAlgebra : 'A library for representing matrices and doing matrix math on arduino' from GitHub user tomstewart98. Available: https://github.com/tomstewart89/BasicLinearAlgebra.git
- adafruit/Adafruit PWM Servo Driver Library: from GitHub user adafruit. Available: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git
- adafruit/Adafruit MPU6050: from GitHub user adafruit. Available: https://github.com/adafruit/Adafruit_MPU6050.git

NOTE: as the PlatformIO project is self-contained, these external libraries do not need to be installed. 

## Setup 
- Clone repository
- Open the firmware/ folder in VSCode with the PlatformIO extension installed
- Connect Arduino Mega 2560 via USB
- Build and upload from the IDE.

## Context

Control loop: 
[controldiagramfin.pdf](https://github.com/user-attachments/files/27321660/controldiagramfin.pdf)
System architecture: [sysarchfin.pdf](https://github.com/user-attachments/files/27321662/sysarchfin.pdf)


