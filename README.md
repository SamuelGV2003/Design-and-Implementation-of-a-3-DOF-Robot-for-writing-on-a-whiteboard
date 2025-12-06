# Design and Implementation of a 3-DOF Robot for writing on a whiteboard
# Robotic Manipulator Project

This repository contains the source code and design files for a robotic manipulator developed as part of an academic project. The system includes trajectory generation in MATLAB, kinematics and communication scripts, firmware for an ESP32-based controller, and CAD drawings of the manufactured parts.

## Overview

The project combines:

- **Trajectory generation** for a planar robotic manipulator.
- **Direct and inverse kinematics** implementation in MATLAB.
- **Serial communication** between MATLAB and the ESP32 controller.
- **Embedded control** of the robot joints using Arduino IDE on an ESP32.
- **Mechanical design** of custom parts used in the physical prototype.

## Repository Structure

- `matlab_trajectories/`  
  MATLAB scripts for generating joint and Cartesian trajectories for the robot.  
  These scripts are used to plan smooth motions that form the outline of the desired letters.

- `matlab_kinematics_communication/`  
  MATLAB code for:
  - Forward and inverse kinematics of the manipulator.
  - Serial communication with the ESP32 to send the computed joint commands.

- `arduino_esp32_control/`  
  Arduino IDE code running on the ESP32 microcontroller.  
  This firmware receives commands from MATLAB and controls the robot actuators (e.g., servomotors, stepper motors) according to the planned trajectories.

- `mechanical_parts_drawings/`  
  CAD files and technical drawings of the main mechanical components of the robot (links, brackets, bases, and other custom parts).  
  These files were used for manufacturing and assembly of the physical prototype.

## Requirements

- **MATLAB** (with basic toolboxes for matrices and plotting).
- **Arduino IDE** with ESP32 board support installed.
- **Serial connection** between the PC (running MATLAB) and the ESP32.
- **CAD viewer** (or CAD software) to open the mechanical drawings.

## How to Use

1. Open the scripts in `matlab_trajectories/` to generate the desired trajectories.
2. Use the code in `matlab_kinematics_communication/` to:
   - Compute the kinematics.
   - Send the corresponding joint commands to the ESP32 via serial communication.
3. Upload the code from `arduino_esp32_control/` to the ESP32 using the Arduino IDE.
4. Refer to the files in `mechanical_parts_drawings/` for details about the physical robot structure and assembly.

## Author

- Students: Samuel Gómez Villanueva, Nelson Vega Barrante and Gabriel Chacón Quesada.
- Institution: Instituto Tecnológico de Costa Rica (TEC)



