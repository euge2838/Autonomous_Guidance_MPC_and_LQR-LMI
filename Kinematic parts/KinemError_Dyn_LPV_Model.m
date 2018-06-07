%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Ac, Bc ] = KinemError_Dyn_LPV_Model (omega_SV,vel_SV,theta_E_SV,delta_SV,alpha_SV) %#codegen
       

    % Vamos a empezar usando delta, pero es posible que haya que hhacer un
    % cambio de variable a sigma como se hizo anteriormente.

    % Vehicle parameters
    M       = 683;
    I       = 561;
    a       = 0.758;
    b       = 1.036;
    g       = 9.81;             
    ro      = 1.2;
    Cf      = 15000;
    Cd      = 0.5;
    Area    = 4;
    mu_roz  = 0.1;
    
    epsilon = 10^-8;
    
    %% Model LPV parameters:

    F_drag  = 0.5*ro*Cd*Area*vel_SV*vel_SV;
    F_rozamiento = mu_roz*M*g;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note:
% This the model used in the MPC-LPV. It has more trigonometric
% commbinations than the used in the IET paper. It is below.

    A1 = -(F_drag+F_rozamiento) / (M*(vel_SV+epsilon));

    A2 = Cf*(sin(delta_SV)*cos(alpha_SV)-sin(alpha_SV)*cos(delta_SV)-sin(alpha_SV)) / M;

    A3 = Cf*(a*(sin(delta_SV)*cos(alpha_SV)-sin(alpha_SV)*cos(delta_SV)) - b*sin(alpha_SV)) / (M*(vel_SV+epsilon));        

    B1 = Cf*(sin(delta_SV)*cos(alpha_SV)-sin(alpha_SV)*cos(delta_SV)) / M;        

    B2 = cos(alpha_SV) / M;
    

    A4 = -Cf*(cos(alpha_SV)*cos(delta_SV)+sin(alpha_SV)*sin(delta_SV)+cos(alpha_SV)) / (M*(vel_SV+epsilon));

    A5 = ( (-Cf*a*(cos(delta_SV)*cos(alpha_SV)+sin(alpha_SV)*sin(delta_SV)) + Cf*b*cos(alpha_SV)) / (M*(vel_SV*vel_SV)) ) - 1;        
    
    B3 = Cf*(cos(alpha_SV)*cos(delta_SV)+sin(alpha_SV)*sin(delta_SV)) / (M*(vel_SV+epsilon));    
     
    B4 = -sin(alpha_SV) / (M*(vel_SV+epsilon));    
    

    A6 = Cf*(b-a*cos(delta_SV)) / I;

    A7 = -Cf*(a*a*cos(delta_SV) + b*b) / (I*(vel_SV+epsilon));    

    B5 = (Cf*a*cos(delta_SV)) / I;

%     A9 = (v_d * sin(theta_E_SV)) / (theta_E_SV+epsilon);
    A9 = (vel_SV * sin(theta_E_SV)) / (theta_E_SV+epsilon); % This should be the reference
%     A10 = (vel_SV * cos(theta_E_SV)) / (theta_E_SV);        % This should be the reference
%     A10 = vel_SV / (theta_E_SV+epsilon);
%     A11 = omega_SV;                                         % This should be the reference
         
    Ac      = [     0     omega_SV  0   -1   0   0;
                -omega_SV    0      A9   0   0   0;
                    0        0      0    0   0   -1;
                    0        0      0    A1  A2  A3;
                    0        0      0    0   A4  A5;
                    0        0      0    0   A6  A7];

    Bc      = [ 0 0;
                0 0;
                0 0;
                B2 B1;
                B4 B3;
                0  B5];
         
    Bref = [ 1 0;
             0 0;
             0 1;
             0 0;
             0 0;
             0 0 ];
         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note:
% This is the model used as dynamic LPV model in the IET paper.
% It is a little bit more simplifyied than the used in the MPC-LPV
% approach.
% % % % %     Cf = 433;
% % % % %     Cr = 367;
%     Cf = 15000;
%     Cr = 15000;

%     A1 = -(F_drag+F_rozamiento)/(M*vel_SV);
%     A2 = Cf*sin(delta_SV)/M;
%     A3 = Cf*sin(delta_SV)*a/(M*vel_SV);
%     A4 = -(Cf*cos(delta_SV)+Cr)/(M*vel_SV); %%% Might be a a mistake
%     A5 = -(Cf*a*cos(delta_SV)-Cr*b)/(M*vel_SV*vel_SV) - 1;
%     A6 = (Cr*b-Cf*a*cos(delta_SV))/I;
%     A7 = (-Cf*a*a*cos(delta_SV) + Cr*b*b)/(I*vel_SV);
%     
%     B2 = 1/M;
%     B1 = Cf*sin(delta_SV)/M;
%     B3 = Cf*cos(delta_SV)/(M*vel_SV);
%     B4 = 0;
%     B5 = Cf*a*cos(delta_SV)/I;
% 
%     A9 = (v_d * sin(theta_E_SV))/(theta_E_SV+epsilon);    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



