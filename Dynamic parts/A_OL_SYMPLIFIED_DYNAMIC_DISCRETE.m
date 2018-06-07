%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ A, B ] = A_OL_SYMPLIFIED_DYNAMIC_DISCRETE( V, ALPHA, DELTA, Ts )
 
    m       = vehicle.m;
    I       = vehicle.I;
    a       = vehicle.a;
    b       = vehicle.b;
    Cf      = vehicle.Cf;
    ro      = vehicle.ro;
    Cd      = vehicle.Cd;
    Area    = vehicle.Area;
    g       = vehicle.g;
    mu_roz  = vehicle.mu_roz;

    % Fuerzas que se oponen
    F_rozamiento = mu_roz*m*g;
    F_drag       = 0.5*ro*Cd*Area*V*V;
      

    %% Matriz A de 6x6 (con filtro añadido e integral part añadida)
    A1 = -(F_drag+F_rozamiento) / (m*V);
    A2 = Cf*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)-sin(ALPHA)) / m;
%     A3 = Cf*(a*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) + b*sin(ALPHA)) / (m*V);
%     B1 = Cf*(-sin(DELTA)*cos(ALPHA)+sin(ALPHA)*cos(DELTA)) / m;
    A3 = Cf*(a*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) - b*sin(ALPHA)) / (m*V);
    B1 = Cf*(+sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) / m;
    
    B2 = cos(ALPHA) / m;
    A4 = -Cf*(cos(ALPHA)*cos(DELTA)+sin(ALPHA)*sin(DELTA)+cos(ALPHA)) / (m*V);
    A5 = ( (-Cf*a*(cos(DELTA)*cos(ALPHA)+sin(ALPHA)*sin(DELTA)) + Cf*b*cos(ALPHA)) / (m*(V*V)) ) - 1;
    B3 = Cf*(cos(ALPHA)*cos(DELTA)+sin(ALPHA)*sin(DELTA)) / (m*V);
    B4 = -sin(ALPHA) / (m*V);
    A6 = Cf*(b-a*cos(DELTA)) / I;
    A7 = -Cf*(a*a*cos(DELTA) + b*b) / (I*V);
    B5 = (Cf*a*cos(DELTA)) / I;

    A = [A1 A2 A3; 
         0 A4 A5; 
         0 A6 A7];
    B = [ B2 B1;
          B4 B3;
          0  B5];

    A = eye(3)+Ts*A;

    B = Ts*B;
end

