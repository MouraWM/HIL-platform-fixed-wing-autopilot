clc; clear; close all;

%% Carregar Piloto Automático Latero-Direcional

%% Ganhos Controle
% Inner Loops
tau = 1.00;
%%

%% Carregar modelo
% Modelo Latero-Direcional
load('Piper J3 1_4 VT0_20.mat');

inputs = {'\delta_a' '\delta_r'};

% Carrega entradas e estados de equilibrio;
equilibrium;

% Dinamica Latero-Direcional do Vant              

%% Washout Filter  
aw= [-1/tau];   bw= [0 1/tau];  
cw= [0;-1];     dw= [1 0; 0 1]; 
wash= ss(aw, bw, cw, dw, 'inputname', inputs);
%%

%% Stability Augmentation System (SAS)
    Kp = 0.119;
    Kr = 0.105;
   
%%

%% Ganhos Controle
Kphi = 1.00;
% PIs
%%

%% Roll-Angle Hold
Kp_phi = -0.090681;
Ki_phi = -0.019958;
Cphi = pid(Kp_phi, Ki_phi);

% [Outer Loop (Phi)]
%%

%% Ganhos Controle
u = Xe(1);
g = 9.80655;
tau1 = 7.5;
%%

%% Carregar modelo
Kpsi = u/(tau1*g);

%%
    
%% Heading Hold 
%%

%% Carregar Piloto Automático Longitudinal

%% Ganhos Controle
% Inner Loops
Ktheta = 1.00;
%%

%% Carregar modelo

% Modelo Longitudinal
load('Piper J3 1_4 VT0_20.mat');

% Carrega entradas e estados de equilibrio;
equilibrium;

% Dinamica Longitudinal do Vant 


%% Stability Augmentation System (SAS)
% Pitch Damper
% Select I/O pair.
    Kq = 0.0877;
        
% [Inner Loop (q)]
%%

%% Pitch-Attitude Hold
% Select I/O pair.
Kp_theta = -0.45319;
Ki_theta = -0.66623;
Ctheta = pid(Kp_theta, Ki_theta);

   
% [Outer Loop (Theta)]
%%
 
%% Ganhos Controle
% Inner Loops
Kh = 1.00;
%%

%% Altitude Hold
% Select I/O pair.
Kp_h = 0.0172410;
Ki_h = 0.0059742;
Ch = pid(Kp_h, Ki_h);


% Realimentacao de [h]
%%


%% Ganhos Controle
% Inner Loops
KVt = 1.00;
% PIs
%%

%% Carregar modelo
%%

%% Speed Hold
% Vt
% Select I/O pair.
Kp_Vt = 0.030269;
Ki_Vt = 0.022003;
CVt = pid(Kp_Vt, Ki_Vt);

% Feedback [Vt]
%%

%% Waypoints(NED)
lambda = 10;

R = 1000;
c1 = R*cos(2*pi/5);
c2 = R*cos(pi/5);
s1 = R*sin(2*pi/5);
s2 = R*sin(4*pi/5);

WP = [ 0    0    400  20; 
	   R    0    400  20; 
	   -c2  s2   420  20; 
	   c1   -s1  380  20; 
	   c1   s1   420  20; 
	   -c2  -s2  380  20; 
	   R    0    400  20]';

clearvars R c1 c2 s1 s2;
clearvars inputs Ki_h Ki_phi Ki_theta Ki_Vt;  
clearvars Kp_h Kp_phi Kp_theta Kp_Vt;
clearvars par_trim wash;
clearvars aw bw cw dw;
clearvars u g tau1;
clearvars Bvt olvt Kp_Vt Ti_Vt;

%%

%% Waypoints(NED)
open PegasusAutopilot;
set_param('PegasusAutopilot','AlgebraicLoopSolver','LineSearch');

%%
