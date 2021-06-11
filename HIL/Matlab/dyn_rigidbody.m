% -----------------------------------------------
% Function - Derivatives of rigid body dynamics
% -----------------------------------------------
% Inputs:
%           1- t: Time
%           2- X: State Variables
%               X(1)  = u
%               X(2)  = v
%               X(3)  = w
%               X(4)  = p
%               X(5)  = q
%               X(6)  = r
%               X(7)  = phi
%               X(8)  = theta
%               X(9)  = psi
%               X(10) = xN
%               X(11) = xE
%               X(12) = xD
%           3- U: Commands
%               U(1)  = dt
%               U(2)  = de
%               U(3)  = da
%               U(4)  = dr
% Output: 
%           1- Xp: State Derivatives
%               Xp(1) = up
%               Xp(2) = vp
%               Xp(3) = wp
%               Xp(4) = pp
%               Xp(5) = qp
%               Xp(6) = rp
%               Xp(7) = phip
%               Xp(8) = thetap
%               Xp(9) = psip
%               Xp(10) = xNp
%               Xp(11) = xEp
%               Xp(12) = xDp
function Xp = dyn_rigidbody(t,X,U,par_gen,par_aero,par_prop)
    %% Input Vector Designation
    u           = X(1);
    v           = X(2);
    w           = X(3);
    aero_var.p	= X(4);
    aero_var.q	= X(5);
    aero_var.r	= X(6);
    phi         = X(7);
    theta       = X(8);
    psi         = X(9);
    xN          = X(10);
    xE          = X(11);
    xD          = X(12);
    
    aero_var.ct = U(1);
    aero_var.ce	= U(2);
    aero_var.ca	= U(3);
    aero_var.cr	= U(4);       
    
    %% Wind    
    wx = 0;
    wy = 0;
    wz = 0;    
    
    %% Atmosphere Model
    [aero_var.rho, ~, ~] = ISA(-xD);
     %T    = 288.15*(1-6.5e-3*(-xD)/288.15);
     %aero_var.rho  = 1013.25e2*(1-6.5e-3*(-xD)/288.15)^(5.2561)/(287.3*T);
    
    
    %% Gravity Model
    gD = 9.80665;
    
    % Transformation Matrix
    Rbn = [cos(theta)*cos(psi)                            cos(theta)*sin(psi)                            -sin(theta)
           sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta)
           cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
    Rnb = Rbn';

    %% Aerodynamic Model
    
    % Aerodynamics Data
    aero_var.V     = sqrt((u-wx)^2 + (v-wy)^2 + (w-wz)^2); 
    aero_var.alpha = atan((w-wz)/(u-wx));
    aero_var.beta  = asin((v-wy)/aero_var.V);   
    
    % Transformation Matrix
    Rwb = [cos(aero_var.alpha)*cos(aero_var.beta)   sin(aero_var.beta)   sin(aero_var.alpha)*cos(aero_var.beta)
          -cos(aero_var.alpha)*sin(aero_var.beta)   cos(aero_var.beta)  -sin(aero_var.alpha)*sin(aero_var.beta)
          -sin(aero_var.alpha)                               0           cos(aero_var.alpha)];
    Rbw = Rwb';
    
    [D, Y, L, LA, MA, NA] = aerodynamics(aero_var, par_aero, par_gen);        
    
    %% Propulsion Model
    
     % Transformation Matrix
    Rpb = [cos(par_prop.aF)*cos(par_prop.bF)   sin(par_prop.bF)   sin(par_prop.aF)*cos(par_prop.bF)
          -cos(par_prop.aF)*sin(par_prop.bF)   cos(par_prop.bF)  -sin(par_prop.aF)*sin(par_prop.bF)
          -sin(par_prop.aF)                               0           cos(par_prop.aF)];
    Rbp = Rpb';    
    
    T = propulsion(aero_var, par_prop, par_aero);    
    
    %% Inertia Model
    Ixx = par_gen.Ixx;
    Iyy = par_gen.Iyy;
    Izz = par_gen.Izz;
    Ixz = -par_gen.Ixz;    
    % Inertia Tensor Matrix
    Ti = [Ixx    0     Ixz
           0     Iyy    0    
          Ixz    0     Izz];
    
    %% Equations of Motion
    
    % Force Equations
    A = Rbw*[-D Y -L]';             % F_Aerodynamic   
    W = Rbn*[0 0 gD]';              % F_Weight
    T = Rbp*T;                      % F_Thrust      
    Fext = ((A + T)/par_gen.m)+ W;	% F_External= F_Aerodynamic + F_Thrust + F_Weight
    
    up = Fext(1) - aero_var.q*w + aero_var.r*v;
    vp = Fext(2) - aero_var.r*u + aero_var.p*w;
    wp = Fext(3) - aero_var.p*v + aero_var.q*u;

    %Rotation Equations
    ppqprp = inv(Ti)*[(LA + (Iyy-Izz)*aero_var.q*aero_var.r + Ixz*aero_var.p*aero_var.q)
                      (MA + (Izz-Ixx)*aero_var.p*aero_var.r + Ixz*(aero_var.r^2-aero_var.p^2))
                      (NA + (Ixx-Iyy)*aero_var.p*aero_var.q - Ixz*aero_var.q*aero_var.r)];

    % Kinematic Equations
    phip   = aero_var.p + aero_var.q*sin(phi)*tan(theta) + aero_var.r*cos(phi)*tan(theta);
    thetap = aero_var.q*cos(phi) - aero_var.r*sin(phi);
    psip   = aero_var.q*sin(phi)/cos(theta) + aero_var.r*cos(phi)/cos(theta);

    % Navigation Equations
    xNpxEpxDp = Rnb*[u v w]';
    
    %% State Derivatives Vector Output
    Xp = [up          % Xp(1)
          vp          % Xp(2)
          wp          % Xp(3)
          ppqprp      % Xp(4,5,6)
          phip        % Xp(7)
          thetap      % Xp(8)
          psip        % Xp(9)
          xNpxEpxDp]; % Xp(10,11,12)
end