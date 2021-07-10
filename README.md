# HIL platform for fixed wing autopilot – A tutorial

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/images/wd.jpg)

![](https://img.shields.io/github/stars/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/forks/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://https://img.shields.io/github/release/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/issues/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/bower/MouraWM/HIL-platform-fixed-wing-autopilot.svg)

## Features

This platform consists of a AP embedded in a dedicated microcontroller to control a simulated fixed-wing aircraft in a Matlab Simulink® programming environment.
The structure allows its adaptation to the needs of new functionalities that want to be introduced. For this purpose, two modules that make up the AP system of a fixed-wing aircraft were loaded into the microcontroller: the control system, responsible for calculating and generating the commands for the aircraft's control surfaces so that it performs a smooth and stable flight; and the guidance system, responsible for generating the reference signals for the aircraft to carry out a predetermined flight mission. 
These modules were implemented from the linearized equations of the mathematical model, in steady state (trimmed), of the fixed wing aircraft model Piper J-3 Cub ¼. 
The navigation module, responsible for reading the sensors, has not been implemented. Instead, it was considered that this module consists of ideal sensors and all the necessary information for the control and guidance modules is provided directly from the simulated aircraft. 
The embedded system was made available for the Arduino platform being tested on Atmega (Arduino Uno), Expressif (Esp8266 and ESP32) and STMicroelectronics (STM32F103) microcontrollers. 
The communication between the HIL and the simulated aircraft takes place through Matlab Simulink® blocks and is made available through of the asynchronous serial communication protocol. 
The necessary steps for connection, configuration, system loading, execution on the microcontroller, modeling and simulation of the aircraft, are described in:

## Autopilot Development for UAV

The development took place in two stages. In the first stage, it was executed communication testing and
configuration of the serial communication protocol between the microcontroller and the simulated aircraft in Matlab
Simulink®. Once configured and tested, the HIL platform's autopilot implementation followed.

Serial communication test: (https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/tree/main/Communication_Tests/Serial_Communication)

HIL Test Platform Implementation: (https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/tree/main/HIL)
<<<<<<< HEAD

=======
>>>>>>> 94e4f9388a6abf73d16e270b061a892e5525508e
