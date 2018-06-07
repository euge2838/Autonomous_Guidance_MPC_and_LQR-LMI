%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef vehicle
    % Vehicle parameters
    properties (Constant)
        m       = 683;
        I       = 561;
        a       = 0.758;
        b       = 1.036;
        Cf      = 24000;
        Cr      = 21000;
        nF      = 2;
        nR      = 2;
        muy     = 0.8;
        g       = 9.81;             
        FzF     = 370 * 9.81;          
        FzR     = 313 * 9.81;
        ro      = 1.2;
        Cd      = 0.5;
        Area    = 4;
        mu_roz  = 0.65;
        
        % Control action constraints:
        % CA
        u_min_ini   = [0; 
                       deg2rad(-0.1)];
        u_max_ini   = [7; 
                       deg2rad(0.1)];   
                 
        u_min       = [0; 
                       -1.4];
        u_max       = [25; 
                       1.4];        
        % derivative of CA     
        du_min_ini  = [-1; 
                       deg2rad(-0.01)];    
        du_max_ini  = [1; 
                       deg2rad(0.01)];
                   
        du_min      = [-10; 
                       -0.3];    
        du_max      = [10; 
                       0.3];       
    end
    
    methods
        function r = initialize(obj,xd,yd,thetad,vd,omegad, option)
            r.x       = xd;
            r.y       = yd;
            r.theta   = thetad;
            r.V       = vd;
            r.Ang_vel = omegad;
            r.AlphaT  = 0;
            if (option==1)
                force   = [obj.m]*[obj.g]*[obj.mu_roz];
                r.t_ini = 6;
            elseif (option==2)
                force   = [obj.m]*[obj.g]*[obj.mu_roz]; %% Cuidado, se tendria que calcular que fuerza tiene esn este instante
                r.t_ini = 0;
            elseif (option==3)
                force   = [obj.m]*[obj.g]*[obj.mu_roz]; %% Cuidado, se tendria que calcular que fuerza tiene esn este instante
                r.t_ini = 0;                
            end
            r.Fxr     = force;
            r.sigma   = atan(r.Ang_vel*[obj.a + obj.b]/(r.V+0.001)) + deg2rad(30);
            r.delta   = atan(r.Ang_vel*[obj.a + obj.b]/(r.V+0.001));
        end
    end
end






