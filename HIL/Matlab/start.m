%% Clear screen, varibles and close all figures 
clc; clear; close all;

%% Load the equilibrium inputs and states

%x = [delta_T delta_e delta_a delta_r]'
Ue = [0.0416; -0.0703;      0;      0];

%x = [u        v       w  p  q  r phi  theta psi pN pE    h]'
Xe = [19.9739; 0; 1.0219; 0; 0; 0; 0; 0.0511; 0; 0; 0; -400];

%% Load Linearized mathematical model of the aircraft [PIPER J-3 CUB 1/4 SCALE]
load('Piper J3 1_4 VT0_20.mat');

%% Open Simulation
open HIL_Serial;
set_param('HIL_Serial','AlgebraicLoopSolver','LineSearch');

