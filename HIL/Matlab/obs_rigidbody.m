% --------------------------------------------------------
% Function - Observation of Rigid Body Dynamics Outputs
% --------------------------------------------------------
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
%           1- Y: Output Vector 
%               Y(1)  = VT
%               Y(2)  = alpha
%               Y(3)  = beta
%               Y(4)  = p
%               Y(5)  = q
%               Y(6)  = r
%               X(7)  = phi
%               X(8)  = theta
%               X(9)  = psi
%               X(10) = xN
%               X(11) = xE
%               X(12) = -xD
function Y = obs_rigidbody(t,X,U)
    %% Input Vector Designation
    u     = X(1);
    v     = X(2);
    w     = X(3);
    p     = X(4);
    q     = X(5);
    r     = X(6);
    phi   = X(7);
    theta = X(8);
    psi   = (X(9));
    xN    = X(10);
    xE    = X(11);
    xD    = X(12);     

    %% Atmosphere, true airspeed, incidences
    % Aerodynamic velocity
    VT    = sqrt(u^2 + v^2 + w^2); 
    % Aerodynamic angles
    alpha = atan(w/u);
    beta  = asin(v/VT);   
       
    %% Output Vector Output
    Y = [VT         % Y(1)
         alpha      % Y(2)
         beta       % Y(3)
         p          % Y(4)
         q          % Y(5)
         r          % Y(6)
         phi        % Y(7)
         theta      % Y(8)
         psi        % Y(9)
         xN         % Y(10)
         xE         % Y(11)
        -xD];       % Y(12)
end