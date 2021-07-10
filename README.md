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


## SYSTEM OF AUTOPILOT for UAV

The autopilot for an autonomous vehicle can be represented as in Figure 1. We are considering the autopilot for a fixed wing aircraft also composed by this generic blocks. These blocks are implemented using electronic components embedded in the aircraft, such as sensors, actuators and microprocessors/microcontrollers, and the software running in them.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig11.png)
> Figure 1. Autopilot blocks.

The Navigation block determines the aircraft's states (position, speed and acceleration) in relation to a given reference frame based on data collected by sensors. This information is treated and made available in navigation data for the Guidance and Control blocks.
The Guidance block determines the best trajectory and/or the reference signals to be reached by the vehicle to execute a desired maneuver. It can be responsible for planning and generating trajectory references to be used by the Control block for the aircraft to execute a desired mission or simple as to compute reference signals to reach predefined waypoints.
The Control block calculates the inputs for the actuators of the moving components (elevator, aileron, rudder and throttle) in order to stabilize the aircraft and effectively execute the desired maneuver.
In this work, the sensors data arrives directly at the Navigation block in an ideal way. With this, the autopilot includes, through programming, the functions of the Guidance and Control block. This implementation was embedded in low-cost microcontrollers from the Arduino platform (Arduino, 2021), being tested on microcontrollers ATmega328P (Arduino UNO) from Microchip (Atmega328p, 2020), Stm32f103c8 from STM Microeletronics (Stm32f103x8, 2015), Esp8266 and Esp32 from Expressif Systems (Esp8266, 2020; Esp32, 2021). For that, the embedded programs have been implemented, in their basic programming standards, for the Serial communication protocol meeting all the microcontrollers mentioned above.
The Aircraft block represents the fixed-wing aircraft to be simulated using Matlab Simulink®. In this work we used the parameters of a Piper J-3 Cub ¼ whose measurements have a quarter of the scale of the original measurements of the Piper J-3 Cub produced by “Piper Aircraft”. 

## Autopilot Development for UAV

The development took place in two stages. In the first stage, it was executed communication testing and
configuration of the serial communication protocol between the microcontroller and the simulated aircraft in Matlab
Simulink®. Once configured and tested, the HIL platform's autopilot implementation followed.

Serial communication test: (https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/tree/main/Communication_Tests/Serial_Communication)

HIL Test Platform Implementation: (https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/tree/main/HIL)

