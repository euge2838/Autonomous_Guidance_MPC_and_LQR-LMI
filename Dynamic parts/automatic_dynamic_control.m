%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef automatic_dynamic_control
    %...
    properties (Constant)
        V_vec       = [2 18];
        Steer_vec   = [deg2rad(-25) deg2rad(25)]; 
        Alpha_vec   = [-0.1 0.1];
        
        sigma_vec   = [deg2rad(-25)+deg2rad(30) deg2rad(25)+deg2rad(30)]; 
        
        steer_min   = -deg2rad(25); 
        steer_max   = deg2rad(25); 
        sigma_min   = -deg2rad(25) + deg2rad(30);
        sigma_max   = deg2rad(25) + deg2rad(30);
        
%         Force_max   = 3500;
%         Force_min   = 0;
        
        Ts          = 0.001;
        gamma       = 0.001;

        Q       = [ 0.05 0 0 0 0 0 0;         %v
                    0 0.01 0 0 0 0 0;         %alpha
                    0 0 0.01 0 0 0 0;         %w
                    0 0 0 0.01 0 0 0;         %Fxr
                    0 0 0 0 10000 0 0;       %delta
                    0 0 0 0 0 90000 0;        %w integral
                    0 0 0 0 0    0 0];       %v integral                
        R       = [0.01  0;
                   0    10];
                           
    end
    
    methods
    end
    
end

