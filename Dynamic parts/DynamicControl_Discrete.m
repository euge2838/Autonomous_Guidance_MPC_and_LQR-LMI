%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Force, Steering_angle, Sigma, poles_checking_D ]...
    = DynamicControl_Discrete( K, x0, controller_inputs, V_ref, Omega_ref, Ts, t)
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the discrete version of the dynamic control. Note that it uses
% the function "A_OL_DYNAMIC_DISCRETE". Moreover, note that the discrete
% model "A,B" has 6 states unlike the continuous system that has 7 states.
%
% Internally the controller works using the sigma variable in order to
% avoid passing through the zero value (singularity). However, when the
% controll goes to apply the control action over the vehicle it must be
% changed to the original variable since the vehicle only understands the
% variable steering.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    persistent integral_omega_error 
    persistent Force_1 Sigma_1
    
%    global vel_min vel_max steer_min steer_max sigma_min sigma_max
    vel_min     = automatic_dynamic_control.V_vec(1);
    vel_max     = automatic_dynamic_control.V_vec(2);
    steer_min   = automatic_dynamic_control.steer_min; 
    steer_max   = automatic_dynamic_control.steer_max; 
    sigma_min   = automatic_dynamic_control.sigma_min; 
    sigma_max   = automatic_dynamic_control.sigma_max; 


    if isempty(integral_omega_error)
        integral_omega_error    = 0;
        Force_1                 = controller_inputs(1);
        Sigma_1                 = controller_inputs(2);
    end
    
    vel         = x0(4);
    alpha       = x0(5);
    omega       = x0(6);
    
    V_despl     = vel + vel_min; % Calculo el control en el rango de velocidad desplazada [Vmin, Vmax] m/s
    
    % Control Inputs:
    Force       = controller_inputs(1);
    Sigma       = controller_inputs(2);
    Steer       = Sigma - deg2rad(30);

    % Polytope Vertexes Controllers:
    k1                  = K(:,:,1);
    k2                  = K(:,:,2);
    k3                  = K(:,:,3);
    k4                  = K(:,:,4);
    k5                  = K(:,:,5);
    k6                  = K(:,:,6);
    k7                  = K(:,:,7);
    k8                  = K(:,:,8);

    % ASÍ ES CORRECTO:
    M_steer_min         = (sigma_max - Sigma)/(sigma_max-sigma_min); 
    M_vel_despl_min     = (vel_max - V_despl)/(vel_max-vel_min);
    M_alpha_min         = (automatic_dynamic_control.Alpha_vec(2) - alpha)/...
                    (automatic_dynamic_control.Alpha_vec(2)-automatic_dynamic_control.Alpha_vec(1));
                
    mu_1                = M_steer_min     * M_vel_despl_min     * M_alpha_min;
    mu_2                = M_steer_min     * M_vel_despl_min     * (1-M_alpha_min);
    mu_3                = M_steer_min     * (1-M_vel_despl_min) * M_alpha_min;
    mu_4                = M_steer_min     * (1-M_vel_despl_min) * (1-M_alpha_min);
    mu_5                = (1-M_steer_min) * M_vel_despl_min     * M_alpha_min;
    mu_6                = (1-M_steer_min) * M_vel_despl_min     * (1-M_alpha_min);
    mu_7                = (1-M_steer_min) * (1-M_vel_despl_min) * M_alpha_min;
    mu_8                = (1-M_steer_min) * (1-M_vel_despl_min) * (1-M_alpha_min);
    
    K_D_interp    = mu_1*k1 + mu_2*k2 + mu_3*k3 + mu_4*k4...
                + mu_5*k5 + mu_6*k6 + mu_7*k7 + mu_8*k8;
            
    [A, B]  = A_OL_DYNAMIC_DISCRETE(vel,alpha,0,Steer,Ts);
    A       = A(1:5,1:5);
    B       = B(1:5,:);
    C       = [1 0; 0 0; 0 1; 0 0; 0 0]'; 

    if sum(isnan(A(:)))>0
        disp('NaN values in Dynamic matrix A')
    end
    
    poles_checking_D = eig(A+B*K_D_interp(:,1:5));

    % Computing feed forward matrix. Source:
    % LIBRO: Eric Ostertag, Mono- and Multivariable Control and Estimation%
    %abs(real(poles_checking_D))<1
    Nbar    = inv(C*inv(eye(5)-B*K_D_interp(:,1:5)-A)*B);  
    
    % Computing integral of the errors:
    omega_error = Omega_ref - omega;
    
    integral_omega_error = integral_omega_error  + omega_error*Ts;
%     if integral_omega_error>0.5
%         integral_omega_error = 0.5;
%     elseif integral_omega_error < -0.5
%         integral_omega_error = -0.5;
%     end
    
    % Feedforward matrix:
    u_new = Nbar * [V_ref Omega_ref]' +...
        K_D_interp*[vel alpha omega Force Sigma integral_omega_error]';        


    % Apkarian filtering:
    %[Force, Sigma] = Apkarian_Filter(u_new, controller_inputs, Ts);
    [Force, Sigma] = Apkarian_Filter(u_new, [Force_1; Sigma_1], Ts);
    Force_1 = Force;
    Sigma_1 = Sigma;
    
    Steering_angle = Sigma - deg2rad(30); % From sigma to delta

    if (Steering_angle > steer_max)
        Steering_angle = steer_max;
    elseif (Steering_angle < steer_min)
        Steering_angle = steer_min;
    end  
        
%     if (Force < 0)
%         Force = 0;
%     elseif (Force > 10000)
%         Force = 10000;
%     end    

end

