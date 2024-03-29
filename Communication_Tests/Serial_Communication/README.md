# HIL platform for fixed wing autopilot – A tutorial

![](https://img.shields.io/github/stars/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/forks/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://https://img.shields.io/github/release/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/issues/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/bower/MouraWM/HIL-platform-fixed-wing-autopilot.svg)


## Serial Communication Test

For the implementation using the serial communication protocol, any microcontroller that meets the Arduino platform can be chosen. For the serial protocol communication and configuration test, it was made available the examples to be used between the microcontroller and the aircraft simulated in Matlab Simulink®. To do this, download the "Serial\_Communication" folder and load the "HIL\_Serial\_Test.ino" file for the chosen microcontroller. After that, the file "Hil\_Serial\_Test.slx", Figure 1, is loaded in Matlab Simulink® environment.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig12.png)
> Figure 1. Simulink block diagram for the serial communication
test (HIL\_Serial\_Test.slx)

At this point, we have the "HIL\_Serial\_Test.ino" file loaded in the
microcontroller and the "HIL\_Serial\_Test.slx" in the Matlab Simulink®
environment. To perform the communication between them we use the
"Serial Configuration", "Serial Receive" and "Serial Send" blocks, which
serve, respectively, to configure the serial port, to receive and to
send data through the selected serial port. Figure 2 shows the settings
made in these blocks.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig17.png)
> Figure 2. Configuration parameters in Matlab Simulink®.

Attention must be paid to the "Communication Port" parameter, which,
depending on the operating system, will indicate the port to be used for
communication. \"COM**x**\" for Windows or \"/dev/ttyUSB**x**\" for
Linux. The letter **x** represents the port number provided by the
operating system. Once the microcontroller is connected, these values
can be selected directly in the "Communication Port" parameter. Other
parameters to be observed are the "Baud Rate", "Header" and
"Terminator", configured with their default options according to Figure 2.

If the communication between the microcontroller and Matlab Simulink® is
working correctly, the graph in Figure will be presented. That is,
considering the Figure , the continuous-time input signal generated by
Matlab Simulink® (Figure 3A), is converted to a digital signal and sent
to the microcontroller. The microcontroller reads this signal and
returns it to the Matlab Simulink®, which displays it on the
oscilloscope (Figure 3B). Note that the returned signal (detailed in the
enlarged image) is a quantized signal.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig16.png)!
> Figure 3. Results presented in the osciloscope: (a) input
signal, (b) signal received from the microcontroller

