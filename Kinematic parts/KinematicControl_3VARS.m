%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ v_control, w_control, poles_checking_K ] = KinematicControl_3VARS( Kc, xD0, v_d, w_d, x_d, y_d, phi_d, V_vec, W_vec)
    %%%%%%%%%%%%%%%%%
    %
    %%%%%%%%%%%%%%%%%
   
%     vel_min_c   = V_vec(1); % Se sobreescribe en la parte dinamica
%     vel_max_c   = V_vec(2);
%     w_min       = W_vec(1); 
%     w_max       = W_vec(2);
    
    %% Vehicle variable bounds
    vel_min_c     = automatic_kinematic_control.V_vec(1);
    vel_max_c     = automatic_kinematic_control.V_vec(2);
    w_min         = automatic_kinematic_control.W_vec(1);
    w_max         = automatic_kinematic_control.W_vec(2); 
    theta_min     = automatic_kinematic_control.Theta_err_vec(1);
    theta_max     = automatic_kinematic_control.Theta_err_vec(2); 
    
    % Computing Errors
    input_err_vec   = [x_d y_d phi_d xD0(1:3)'];
    pose_errors     = errors_Lyapunov(input_err_vec);
    
    % Computing LPV Control Actions
    V_despl     = xD0(4) + vel_min_c;
    %     V_despl     = xD0(4);
    W           = xD0(6);
    THETA_ERR   = pose_errors(3);
    
    k1          = Kc(:,:,1);    k2          = Kc(:,:,2);
    k3          = Kc(:,:,3);    k4          = Kc(:,:,4);
    k5          = Kc(:,:,5);    k6          = Kc(:,:,6);
    k7          = Kc(:,:,7);    k8          = Kc(:,:,8);
    
    M_w_min     = (W - w_min)/(w_max-w_min);
    M_vel_min   = (V_despl - vel_min_c)/(vel_max_c-vel_min_c);
    M_theta_min = (THETA_ERR - theta_min)/(theta_max-theta_min);
    
    mu_1        = M_w_min * M_vel_min * M_theta_min;
    mu_2        = M_w_min * M_vel_min * (1-M_theta_min);
    mu_3        = M_w_min * (1-M_vel_min) * M_theta_min;
    mu_4        = M_w_min * (1-M_vel_min) * (1-M_theta_min);
    mu_5        = (1-M_w_min) * M_vel_min * M_theta_min;
    mu_6        = (1-M_w_min) * M_vel_min * (1-M_theta_min);
    mu_7        = (1-M_w_min) * (1-M_vel_min) * M_theta_min;
    mu_8        = (1-M_w_min) * (1-M_vel_min) *(1-M_theta_min);

    K_interp_c    = mu_1*k1 + mu_2*k2 + mu_3*k3 + mu_4*k4 + ...
                  mu_5*k5 + mu_6*k6 + mu_7*k7 + mu_8*k8;
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Checking poles
    [A_checking, B_checking] =  A_OL_KINEMATIC(V_despl,W);
    poles_checking_K = eig(A_checking+B_checking*K_interp_c);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%% New  control law (new IET paper submission)%%%%%
    u = K_interp_c*pose_errors + [v_d*cos(THETA_ERR) w_d]';

    if (u(2) > w_max)
        u(2) = w_max;
    elseif (u(2) < w_min)
        u(2) = w_min;
    end
    v_control = u(1);
    w_control = u(2);
end

