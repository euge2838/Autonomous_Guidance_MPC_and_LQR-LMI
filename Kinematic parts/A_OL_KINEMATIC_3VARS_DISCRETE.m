%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ A, B ] = A_OL_KINEMATIC_3VARS_DISCRETE( vd, omega, theta_Err, Ts )

    A3by3 = [1        omega*Ts   0; 
            -omega*Ts   1     vd*(sin(theta_Err)/theta_Err)*Ts; 
             0          0     1];

    B3by2 = [-Ts 0;
              0 0; 
              0 -Ts];
    A = A3by3;
    B = B3by2;
end

