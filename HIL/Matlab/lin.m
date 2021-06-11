% -----------------------------------------------
% Function - Aircraft Linearization
% -----------------------------------------------
% Inputs:
%           1- Xe: Equilibrium States
%           2- Ue: Equilibrium Inputs
% Output: 
%           1- A: State Space A Matrix
%           2- B: State Space B Matrix
%           3- C: State Space C Matrix
%           4- D: State Space D Matrix

function [A,B,C,D] = lin(Xe, Ue, par_gen, par_aero, par_prop)
    % Matrix Sizes
    n = size(Xe,1);
    m = size(Ue,1);

    % A Matrix
    for j = 1:n
        dxj    = zeros(n,1);
        dxj(j) = Xe(j)*0.025 + eps; % eps= Computer epsilon
        Xpup   = dyn_rigidbody(0, Xe+dxj, Ue, par_gen, par_aero, par_prop);
        Xpdw   = dyn_rigidbody(0, Xe-dxj, Ue, par_gen, par_aero, par_prop);
        A(:,j) = (Xpup-Xpdw)/(2*dxj(j));
    end

    % B Matrix
    for j = 1:m
        duj    = zeros(m,1);
        duj(j) = Ue(j)*0.025 + eps; % eps= Computer epsilon 
        Xpup   = dyn_rigidbody(0, Xe, Ue+duj, par_gen, par_aero, par_prop);
        Xpdw   = dyn_rigidbody(0, Xe, Ue-duj, par_gen, par_aero, par_prop);
        B(:,j) = (Xpup-Xpdw)/(2*duj(j));
    end

    % C Matrix
    for j = 1:n
        dxj    = zeros(n,1);
        dxj(j) = Xe(j)*0.025 + eps; % eps= Computer epsilon 
        Yup   = obs_rigidbody(0, Xe+dxj, Ue);
        Ydw   = obs_rigidbody(0, Xe-dxj, Ue);
        C(:,j) = (Yup-Ydw)/(2*dxj(j));
    end

    % D Matrix
    for j = 1:m
        duj    = zeros(m,1);
        duj(j) = Ue(j)*0.025 + eps; % eps= Computer epsilon 
        Yup   = obs_rigidbody(0, Xe, Ue+duj);
        Ydw   = obs_rigidbody(0, Xe, Ue-duj);
        D(:,j) = (Yup-Ydw)/(2*duj(j));
    end
end