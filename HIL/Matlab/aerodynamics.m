% aerodynamic model
% INPUTS:
% aero_var: structure with following fields
%   -> rho: air density
%   -> V: TAS
%   -> alpha: angle of attack
%   -> beta: sideslip angle
%   -> p: roll rate
%   -> q: pitch rate
%   -> r: yaw rate
%   -> ca: aileron deflection
%   -> ce: elevator deflection
%   -> cr: rudder deflection
% par_aero: parameters of aerodynamic model (generated by DataA300)
% par_gen: general parameters (generated by DataA300)
function [D,Y,L,LA,MA,NA]=aerodynamics(aero_var,par_aero,par_gen)

% lift coefficient:
CL=par_aero.CL.null+...
    par_aero.CL.alpha*aero_var.alpha+...
    par_aero.CL.q*(aero_var.q*par_gen.c/par_aero.Vref);
     %par_aero.CL.ce*aero_var.ce;

% drag coefficient:
CD=par_aero.CD.null+...
    CL^2/(pi*0.8*(par_gen.b/par_gen.c));

% pitch moment coefficient:
Cm=par_aero.Cm.null+...
    par_aero.Cm.alpha*aero_var.alpha+...
    par_aero.Cm.q*(aero_var.q*par_gen.c/par_aero.Vref)+...
    par_aero.Cm.ce*aero_var.ce;

% lateral force coefficient:
CY=par_aero.CY.beta*aero_var.beta+...
    par_aero.CY.p*(aero_var.p*par_gen.b/par_aero.Vref)+...
    par_aero.CY.r*(aero_var.r*par_gen.b/par_aero.Vref)+...
    par_aero.CY.ca*aero_var.ca+...
    par_aero.CY.cr*aero_var.cr;

% roll moment coefficient:
Cl=par_aero.Cl.beta*aero_var.beta+...
    par_aero.Cl.p*(aero_var.p*par_gen.b/par_aero.Vref)+...
    par_aero.Cl.r*(aero_var.r*par_gen.b/par_aero.Vref)+...
    par_aero.Cl.ca*aero_var.ca+...
    par_aero.Cl.cr*aero_var.cr;

% yaw moment coefficient:
Cn=par_aero.Cn.beta*aero_var.beta+...
    par_aero.Cn.p*(aero_var.p*par_gen.b/par_aero.Vref)+...
    par_aero.Cn.r*(aero_var.r*par_gen.b/par_aero.Vref)+...
    par_aero.Cn.ca*aero_var.ca+...
    par_aero.Cn.cr*aero_var.cr;

% longitudinal aerodynamics
L=0.5*aero_var.rho*aero_var.V^2*par_gen.S*CL; % lift
D=0.5*aero_var.rho*aero_var.V^2*par_gen.S*CD; % drag
MA=0.5*aero_var.rho*aero_var.V^2*par_gen.S*par_gen.c*Cm; % pitch moment

% lateral-directional aerodynamics
Y=0.5*aero_var.rho*aero_var.V^2*par_gen.S*CY; % lateral force
LA=0.5*aero_var.rho*aero_var.V^2*par_gen.S*par_gen.b*Cl; % roll moment
NA=0.5*aero_var.rho*aero_var.V^2*par_gen.S*par_gen.b*Cn; % roll moment
