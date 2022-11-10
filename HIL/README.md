# HIL platform for fixed wing autopilot – A tutorial

![](https://img.shields.io/github/stars/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/forks/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://https://img.shields.io/github/release/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/issues/MouraWM/HIL-platform-fixed-wing-autopilot.svg) ![](https://img.shields.io/github/bower/MouraWM/HIL-platform-fixed-wing-autopilot.svg)

## HIL Test Platform Implementation

In this folder, there are two subfolders: "Arduino", where the necessary
files to be embedded in the microcontroller are found; and "Matlab",
where the files used for the simulation of the fixed-wing aircraft are
found.

In the first folder there are three files: "HIL\_Serial.ino", which
describes the implementation of the Control and Guidance block; the file
"HIL\_Waypoints.h", which defines the waypoints to be reached by the
autopilot; and the file "HIL\_Variables.h", which defines the variables
used by the program.

The mapping of the desired waypoints, for each trajectory, must be
provided in a matrix of points defined in the local navigation
coordinate system (NED). The Table 1 shows the tested example.

|     | ![](https://render.githubusercontent.com/render/math?math=\large{WP_1}) | ![](https://render.githubusercontent.com/render/math?math=\large{WP_2}) | ![](https://render.githubusercontent.com/render/math?math=\large{WP_3}) | ![](https://render.githubusercontent.com/render/math?math=\large{WP_4}) | ![](https://render.githubusercontent.com/render/math?math=\large{WP_5}) | ![](https://render.githubusercontent.com/render/math?math=\large{WP_6}) | ![](https://render.githubusercontent.com/render/math?math=\large{WP_7}) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|![](https://render.githubusercontent.com/render/math?math=\large{N\(m\)}) |0|1000|-809.0170|309.0170|309.0170|-809.0170|1000|
|![](https://render.githubusercontent.com/render/math?math=\large{E\(m\)}) |0|0|587.7853|-951.0565|951.0565|-587.7853|0|
|![](https://render.githubusercontent.com/render/math?math=\large{D\(m\)}) |400|400|420|380|420|380|400|
|![](https://render.githubusercontent.com/render/math?math=\large{V\(m/s\)}) |20|20|20|20|20|20|20|
> Table 1. Waypoints mapping.

These files must be compiled on the Arduino platform and embedded in the
chosen microcontroller.

In the second folder, there are the Matlab Simulink® files to be used in
the simulation.

The "start.m" file initializes the parameters and file necessary to the
execution of the simulate aircraft, in this case the Piper J-3 Cub ¼. It
defines the input vectors and aircraft steady states determined
according to Equation below; loads the file "Piper J3 1\_4 VT0\_20.mat" which
contains the parameters of the linearized model of the fixed-wing
aircraft used to be simulated; and opens the file "HIL\_Serial.slx".

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig26.png)

This last file, "HIL\_Serial.slx", contains the Matlab Simulink® blocks,
shown in Figure 1, and it is responsible for performing the simulation.
It is composed of the "HIL (Serial) - Navigation / Guidance / Control"
block, the "Aircraft" block and the state and input vectors (defined in
Equation above) calculated on each interaction.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig18.png)
> Figure 1. Matlab Simulink® blocks from the
"HIL\_Serial.slx" file.

The "HIL (Serial) - Navigation / Guidance / Control" block (Figure 2) is
responsible for the communication between the microcontroller and the
simulated aircraft. In the first step, the content of the input vector
(![](https://render.githubusercontent.com/render/math?math=\large{\vec{y}})) is sent to the microcontroller through the serial communication
protocol. Once received, the microcontroller performs the calculations
and returns, for the simulation, the content of the state vector
(![](https://render.githubusercontent.com/render/math?math=\large{\vec{du}}))
\- which is sent to the "Aircraft" block; a stop signal, triggered
when the simulated aircraft reaches the final waypoint; and the altitude
(![](https://render.githubusercontent.com/render/math?math=\large{H_e}))
and yaw angle (![](https://render.githubusercontent.com/render/math?math=\large{\psi_e})) reference signals.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig19.png)
> Figure 2. Subsystem "HIL (Serial) -- Navigation / Guidance
/ Control" detail.

The configurations of the "Serial Configuration" and "Serial Send"
blocks follow the same configuration reported in Figure 2 of "Serial Communication Test" in GitHub (https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/tree/main/Communication_Tests/Serial_Communication).
For the "Serial Receive" block, the "Data Size" parameter must be set to
a seven-position vector, as shown in Figure 3.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig20.png)
> Figure 3. Configuration of the "Serial Receive" block.

Finally, the "Aircraft" block (see Figure 1) is implemented by the
subsystem shown in Figure 4. This block receives the state vector (![](https://render.githubusercontent.com/render/math?math=\large{\vec{d_u}}))
coming from the "HIL (Serial) - Navigation / Guidance / Control" block
and adds it to the steady state vector (![](https://render.githubusercontent.com/render/math?math=\large{\vec{U_e}})), defined in "start.m",
generating the state vector to be used in the function
"sfunction\_piper.m". This function performs the calculations of the
rigid body dynamics of the fixed-wing aircraft generating the new values
for the input vector, repeating all the cycle.

Once the simulation is finished, the graphics for the analysis of the
output signals (trajectory, speed, altitude and yaw angle) can be
generated through the file \"plot\_graphics.m\".

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig21.png)
> Figure 4. Detail of the Aircraft subsystem.

## Results and Discussions

Mission results consist of traversing a trajectory of predefined
waypoints. After reaching the last desired point, the mission is
considered complete and the simulation ends.

For the simulation, the star-shaped trajectory defined in Table 1 was
selected, where the waypoints are defined in the NED coordinate system. The microcontroller
used was ESP32 of Expressif Systems.

During the simulations, the reference speed of the fixed-wing aircraft
was kept in the steady region ![](https://render.githubusercontent.com/render/math?math=\large{(V_{T_r}=20m\/s)}).

The simulation was considered to take place under ideal conditions.

Figure 5 shows the result of the trajectory obtained. Here we see that
the aircraft went through the preset waypoints starting at WP1 and
ending at WP7. The duration of the mission was 562s.

Figure 6 shows the responses for the airspeed. 

Figures 7 and 8 show the responses, respectively, for the altitude and
yaw angle of the fixed-wing aircraft during the simulation.

Observing the graphics we see that the waypoints were reached thus showing
the success of the mission.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig22.png)
> Figure 5. Trajectory performed by aircraft in simulation.

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig23.png)
> Figure 6. Speed performed by aircraft in simulation. 

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig24.png)
> Figure 7. Altitude performed by aircraft in simulation. 

![](https://github.com/MouraWM/HIL-platform-fixed-wing-autopilot/blob/main/images/Fig25.png)
> Figure 8. Angle performed by aircraft in simulation.

