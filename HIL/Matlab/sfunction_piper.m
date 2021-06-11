% -----------------------------------------------
% Function - Piper J Cub 1/4 Equations of Motion
% -----------------------------------------------
% Inputs:
%           1- t,x,u,flag,Xe: Model Initilization of s-function with Aircraft States at Equilibrium 
%               Xe(1)  = u_e
%               Xe(2)  = v_e
%               Xe(3)  = w_e
%               Xe(4)  = p_e
%               Xe(5)  = q_e
%               Xe(6)  = r_e
%               Xe(7)  = phi_e
%               Xe(8)  = theta_e
%               Xe(9)  = psi_e
%               Xe(10) = xN_e
%               Xe(11) = xE_e
%               Xe(12) = xD_e
% Output: 
%           case 0 (Initialization):
%               1- sys,x0,str,ts,simStateCompliance: Model Initialization with Aircraft States at Equilibrium
%           case 1 (Derivatives):
%               1- sys: State Derivatives
%                   sys(1) = up
%                   sys(2) = vp
%                   sys(3) = wp
%                   sys(4) = pp
%                   sys(5) = qp
%                   sys(6) = rp
%                   sys(7) = phip
%                   sys(8) = thetap
%                   sys(9) = psip
%                   sys(10) = xNp
%                   sys(11) = xEp
%                   sys(12) = xDp
%           case 3 (Output):
%               1- sys: Output Vector
%                   sys(1)  = VT
%                   sys(2)  = alpha
%                   sys(3)  = beta
%                   sys(4)  = p
%                   sys(5)  = q
%                   sys(6)  = r
%                   sys(7)  = phi
%                   sys(8)  = theta
%                   sys(9)  = psi
%                   sys(10) = xN
%                   sys(11) = xE
%                   sys(12) = -xD
function [sys,x0,str,ts,simStateCompliance]=sfunction_piper(t,x,u,flag,par_gen,par_aero,par_prop,Xe) % Do not delete or change t,x,u,flag and sys,x0,str,ts
    switch flag
        %%%%%%%%%%%%%%%%%%
        % Initialization %
        %%%%%%%%%%%%%%%%%%
        case 0         
            [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe);

        %%%%%%%%%%%%%%%
        % Derivatives %
        %%%%%%%%%%%%%%%
        case 1
            sys = dyn_rigidbody(t,x,u,par_gen,par_aero,par_prop);

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Update and Terminate %
        %%%%%%%%%%%%%%%%%%%%%%%%
        case {2,9}
            sys = []; % do nothing

        %%%%%%%%%%
        % Output %
        %%%%%%%%%%
        case 3
            sys = obs_rigidbody(t,x,u); 

        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end



% ---------------------------------------------------------------------------------------
% Function - Return the sizes, initial conditions, and sample times for the S-function.
% ---------------------------------------------------------------------------------------
% Inputs:
%           1- Xe: Aircraft States at Equilibrium
%               Xe(1)  = u_e
%               Xe(2)  = v_e
%               Xe(3)  = w_e
%               Xe(4)  = p_e
%               Xe(5)  = q_e
%               Xe(6)  = r_e
%               Xe(7)  = phi_e
%               Xe(8)  = theta_e
%               Xe(9)  = psi_e
%               Xe(10) = xN_e
%               Xe(11) = xE_e
%               Xe(12) = xD_e
% Output: 
%           1- sys,x0,str,ts,simStateCompliance: Model Initialization at the States inputted

function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(Xe)
    sizes = simsizes;
    sizes.NumContStates  = 12;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 12;
    sizes.NumInputs      = 4;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    str = [];
    x0  = Xe;
    ts  = [0 0];   % sample time: [period, offset]

    % specify that the simState for this s-function is same as the default
    simStateCompliance = 'DefaultSimState';

end % mdlInitializeSizes