# HIL platform for fixed wing autopilot – A tutorial

![](https://img.shields.io/github/stars/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/forks/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://https://img.shields.io/github/release/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/issues/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/bower/MouraWM/HIL-platform-fixed-wing-autopilot.svg)

## HIL Test Platform Implementation
This tutorial is composed by the source files of the implementation of a fixed-wing aircraft HIL control platform whose content was accepted for publication in the 26th COBEM.

## Features

There are two subfolders: "Arduino", where the files needed to be embedded in the microcontroller were developed; and "Matlab", where the files used to simulate fixed-wing aircraft were developed.

In the first folder there are three files: "HIL\_Serial.ino", which describes the implementation of the Control and Guidance block; the file "HIL\_Waypoints.h", which defines the waypoints to be reached by the autopilot; and the file "HIL\_Variables.h", which defines the variables used by the program. All these files must be compiled and embedded in the chosen microcontroller.

In the second folder are the Matlab Simulink® files to be used in the simulation. The "start.m" file initializes the necessary parameters for the execution of the aircraft simulation and opens the "HIL\_Serial.slx" file that contains the Matlab Simulink® blocks responsible for carrying out the simulation.
