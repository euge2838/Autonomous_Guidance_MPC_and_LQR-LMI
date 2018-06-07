%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef automatic_kinematic_control
    
    properties(Constant)
        V_vec           = [1 18];  % Case of State FEedback LPV kinematic control
%         V_vec           = [2 18]; % Case of MPC kinematic control

        Theta_err_vec   = [deg2rad(-5) deg2rad(5)];
        W_vec           = [-1.417 1.417];
        gamma = 0.01;
        Q               = [ 2    0     0;
                            0    3     0;
                            0    0    20];
        R               = [ 0.5   0; 
                            0   0.001];
    end
    
    methods
    end
    
end

