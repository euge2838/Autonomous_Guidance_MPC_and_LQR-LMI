%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ A, B ] = A_OL_DYNAMIC_DISCRETE( V, ALPHA, dPSI, DELTA, Ts )
 
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
    
    %%% Apkarian Filter
    A_filter     = [-10 0; 0 -10];

    % Fuerzas que se oponen
    F_rozamiento = mu_roz*m*g;
    F_drag       = 0.5*ro*Cd*Area*V*V;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Solo usar en caso de que SIGMA sea la input (pero no es necesario).
%     DELTA = SIGMA-deg2rad(30);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    %% Matriz A de 6x6 (con filtro añadido e integral part añadida)
    A1 = -(F_drag+F_rozamiento) / (m*V);    
    A2 = Cf*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)-sin(ALPHA)) / m;
%     A3 = Cf*(a*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) + b*sin(ALPHA)) / (m*V);        
%     B1 = Cf*(-sin(DELTA)*cos(ALPHA)+sin(ALPHA)*cos(DELTA)) / m;        
    A3 = Cf*(a*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) - b*sin(ALPHA)) / (m*V);        
    B1 = Cf*(sin(DELTA)*cos(ALPHA)-sin(ALPHA)*cos(DELTA)) / m; 
    
    B2 = cos(ALPHA) / m;
    A4 = -Cf*(cos(ALPHA)*cos(DELTA)+sin(ALPHA)*sin(DELTA)+cos(ALPHA)) / (m*V);
    A5 = ( (-Cf*a*(cos(DELTA)*cos(ALPHA)+sin(ALPHA)*sin(DELTA)) + Cf*b*cos(ALPHA)) / (m*(V*V)) ) - 1;        
    B3 = Cf*(cos(ALPHA)*cos(DELTA)+sin(ALPHA)*sin(DELTA)) / (m*V);    
    B4 = -sin(ALPHA) / (m*V);    
    A6 = Cf*(b-a*cos(DELTA)) / I;
    A7 = -Cf*(a*a*cos(DELTA) + b*b) / (I*V);    
    B5 = (Cf*a*cos(DELTA)) / I;

    A3by3 = [A1 A2 A3; 
             0 A4 A5; 
             0 A6 A7];
    B3by2 = [ B2 B1;
              B4 B3;
              0  B5];

    A =  [A3by3      B3by2    zeros(3,1);
         zeros(2,3) A_filter  zeros(2,1);
         0 0 -1      0 0           0 ];
    A = eye(6)+Ts*A;

    B = [0 0; 
         0 0; 
         0 0; 
         10 0; 
         0 10;
         0  0];
    B = Ts*B;
end

