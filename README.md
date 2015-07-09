## InRoL Quadrotor Platform for Research

This is a repository for the system developed as a quadrotor research platform for interactive / network robotics laboratory in Seoul Natinal University

## Overview
The system is built in two versions. USB2PPm system and ROS System. Each system consists of several subsystems as below. 
- USB2PPM System
  - PC USB2PPM interface 
  - USB2PPM
  - Quadrotor

- ROS System
  - GCS
  - Quadrotor

## USB2PPM System
### PC USB2PPM interface software
PC USB2PPM interface is the ground control software which interfaces with the USB2PPM. It is a library which provides functions to send RC commands throught the trainer port of the transmitter. Turnigy 9X is used as a transmitter in the current system. 

### USB2PPM
USB2PPM is a hardware developed to be enable the PC to interface with the Turnigy 9X transmitter. The USB2PPM receives commands through USB connection with the PC(UART Communication) and transmitts PPM signals to the trainer port of Turnigy 9X

### Quadrotor
A custom quadrotor was developed to seve variable purposes such as aerial manipulation .etc The quadrotor is powered by a NAZE 32 flight controller.
