%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Ac, Bc, Bref ] = KinemError_LPV_Model (KC, Hp, theta_E_SV, omega_SV, v_d_SV) %#codegen
    
    epsilon = 10^-8;
    
    Ac  = zeros(3,3,Hp);
    Bc  = zeros(3,2,Hp);
    
    %% Model LPV parameters:
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note:
% This the model used in the MPC-LPV. It has more trigonometric
% commbinations than the used in the IET paper.

    A9 = (v_d_SV * sin(theta_E_SV)) / (theta_E_SV+epsilon);
     
    %% LPV State Space model
    Ac_pred = [     0     omega_SV  0 ;
                -omega_SV    0      A9;
                    0        0      0 ];

    Bc_pred = [ -1 0;
                0 0;
                0 -1];
         
    Bref = [ 1 0;
             0 0;
             0 1];
         
    %% La primera iteracion de la simulacion es diferente.
    if (KC==1)
% %     if (t>0)        
        for i=1:Hp
            Ac(:,:,i) = Ac_pred;
            Bc(:,:,i) = Bc_pred;
        end
    else
        Ac = Ac_pred;
        Bc = Bc_pred;
    end




