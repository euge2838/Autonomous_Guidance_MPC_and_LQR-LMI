%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Fy = Fy_Tire_Model(alpha, Fz, muy)
    % Input
    % alpha - slip angle [rad]
    % Fz    - Load [N]
    % muy   - Lateral friction coefficient (*1000) [-]

    % Slip angle treatment
    ALPHA = asin(sin(alpha)); % [rad]
    ALPHA = 180 / pi * ALPHA; % Conversion [rad] - [deg]
    % Nominal parameters
    a0  = 1;
    a1  = 0;
    a2  = 800;
    a3  = 3000;
    a4  = 50;
    a5  = 0;
    a6  = 0;
    a7  = -1;
    a8  = 0;
    a9  = 0;
    a10 = 0;
    a11 = 0;
    a12 = 0;
    a13 = 0;
    
    Fz = Fz/1000;           % Conversion [N] - [kN]

    camber = 0;             % Camber angle

    C = a0;                 % Shape factor
    muy0 = a1 * Fz + a2;      % Lateral friction coefficient nominal [-]
    muy = muy * 1000;         % Lateral friction coefficient operacional
    D = muy0 * Fz;            % muy = lateral friction coefficient
    BCD = a3 * sin(2 * atan(Fz/a4))*(1-a5 * abs(camber)); % Cornering stiffness
    E = a6 * Fz + a7;         % Curvature factor
    B = BCD/(C * D);          % stiffness factor
    Sh = a8 * camber + a9 * Fz + a10;      % Horizontal shift
    Sv = a11 * Fz * camber + a12 * Fz + a13; % Vertical shift
    ALPHAeq = muy0/muy*(ALPHA + Sh);   % Equivalent slip angle
    
% Reference characteristics
    fy = D * sin(C * atan(B * ALPHAeq - E*(B * ALPHAeq - atan(B * ALPHAeq))));
% Lateral force
    Fy = -muy/muy0*(fy + Sv);
end