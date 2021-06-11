% -------------------------------------------------------------------------
% Decoupling of longitudinal and lateral-directional dynamics
% -------------------------------------------------------------------------
% Author: 
% Marcelo Henrique dos Santos, ITA, São José dos Campos, SP
%
% Parameters:
% * mode: [1] - Longitudinal
%         [2] - Latero-Directional
%
% * Returns de state space matrices of the UAV
%   [A,B,C,D] = decoupling(mode) returns the state space matrices A, B, 
%               C and D of the UAV.
%%

function [A,B,C,D] = decoupling( mode, par_gen, par_aero, par_prop )

    %% Indexes    
    %States 
    idx = [1 3 5 8 12;
           2 4 6 7 9];
    %Inputs 
    idu = [1 2;
           3 4];
    %Outputs 
    idy = [1 2 5 8 12;
           3 4 6 7 9];    

    %% Initialization of Matrices    
    A = zeros(length(idx));
    B = zeros(length(idx), length(idu));
    C = zeros(length(idy));
    D = zeros(length(idx), length(idu));

    %% Linearization of the nonlinear model
    % Load inputs and states of equilibrium;
    equilibrium;

    % Linearization
    [Ax, Bx, Cx, Dx] = lin(Xe, Ue, par_gen, par_aero, par_prop);

    %% Desacoplamento
    
    % Longitudinal
    if mode == 1
        %x = [u w q theta h]'    
        for i=1:length(idx)
            for j=1:length(idx)
                A(i,j) = Ax(idx(mode, i), idx(mode,j)); 
            end        
        end

        %u = [delta_th delta_e]'    
        for i=1:length(idu)
            for j=1:length(idx)
                B(j,i) = Bx(idx(mode, j), idu(mode,i)); 
            end        
        end  

        %y = [Vk alpha q theta h]'
        for i=1:length(idy)
            for j=1:length(idy)
                C(i,j) = Cx(idy(mode, i), idx(mode,j)); 
            end        
        end

    % Latero-Direcional
    elseif mode == 2
        %x = [v p r phi psi]'
        for i=1:length(idx)
            for j=1:length(idx)
                A(i,j) = Ax(idx(mode, i), idx(mode,j)); 
            end        
        end

        %u = [delta_a delta_r]' 
        for i=1:length(idu)
            for j=1:length(idx)
                B(j,i) = Bx(idx(mode, j), idu(mode,i)); 
            end        
        end 
		
        %y = [beta p r phi psi]'  
        for i=1:length(idy)
            for j=1:length(idy)
                C(i,j) = Cx(idy(mode, i), idx(mode,j)); 
            end        
        end
    else
        disp('Erro na escolha da dinâmica do UAV.')
    end
    clear Ax Bx Cx idx idy idu;
end

