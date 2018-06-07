%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = dynamic_complex_model(t, states, u)
    
% % %     % SEAT IRI Vehicle Parameters    
% % %     m       = 1500 + 2*80 + 300; % mass, kg
% % %     Area    = 2.2;      % Superficie frontal, m^2
% % %     Cd      = 0.3;      % Cd aerodinamico
% % %     g       = 9.81;     % Gravity
% % %     ro      = 1.225;    % Densidad del aire en SI    
% % %     a       = 4;
% % %     b       = 1.8;
% % %     I       = m*(a*a + b*b)/12;
% % %     FzF     = m/2 * 9.81;          
% % %     FzR     = m/2 * 9.81;
% % %     muy     = 0.8;

    % CVC Vehicle Parameters    
    m       = vehicle.m;
    I       = vehicle.I;
    a       = vehicle.a;
    b       = vehicle.b;
    ro      = vehicle.ro;
    Cd      = vehicle.Cd;
    Area    = vehicle.Area;
    g       = vehicle.g;
    muy     = vehicle.muy;
    FzF     = vehicle.FzF;
    FzR     = vehicle.FzR;

    mu_friction  = u(3);
    F_friction = mu_friction*m*g;  % Fuerzas que se oponen
    
    % States
    X       = states(1);
    Y       = states(2);
    PSI     = states(3);
    V       = states(4);        % Velocity [m/s]
    ALPHAT  = states(5);        % Slip angle [rad]
    OMEGA   = states(6);        % Yaw rate [rad/s]

    DELTA  = u(2);

    % Slip angles
    ALPHAF = atan2((V * sin(ALPHAT) + a * OMEGA), (V * cos(ALPHAT))) - DELTA; 
    ALPHAR = atan2((V * sin(ALPHAT) - b * OMEGA), (V * cos(ALPHAT)));         

    %%% Lateral forces (non-linear model)
    FyF = Fy_Tire_Model(ALPHAF, FzF, muy);
    FyR = Fy_Tire_Model(ALPHAR, FzR, muy);
 
%     Cf      = 15000;
%     FyF = Cf*(DELTA-ALPHAT-(a*OMEGA/V))
%     FyR = Cf*(-ALPHAT+(b*OMEGA/V))

    FxR = u(1); 

    F_drag  = 0.5*ro*Cd*Area*V*V;

    % Equations of motion:
    dx(1,1) = V * cos(ALPHAT + PSI);    % X
    dx(2,1) = V * sin(ALPHAT + PSI);    % Y
    dx(3,1) = OMEGA;                     % dPSI
    dx(4,1) =  ( FxR * cos(ALPHAT) + FyF * sin(ALPHAT - DELTA) + FyR * sin(ALPHAT)...
                - F_drag - F_friction )/m ;                          %dV
    dx(5,1) = ( - FxR * sin(ALPHAT) +...
        FyF * cos(ALPHAT - DELTA) + FyR * cos(ALPHAT) - m * V * OMEGA) / (m * V);    %dALPHAT
    dx(6,1) = (FyF * a * cos(DELTA) - FyR * b) / I;          % Theta acceleration
