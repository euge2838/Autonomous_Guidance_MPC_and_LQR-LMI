%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ u_opt, u_predicted, x_predicted, x_err] = NLMPC_Kinematic_Computation...
    ( controller, Hp, KC, oldu, u_predicted, x_predicted, x_Real,...
    kin_states, kin_inputs, x_Ref, y_Ref, theta_Ref, vel_Ref, omega_Ref,...
    longitud_vect, car)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%\\ Vehicle kinematic control:
%   controller: object provided by the MPC design
%   Hp: Prediction horizon
%   KC: discrete kinematic time
%   oldu: previous control action
%   u_predicted: vector of predicted control actions in the last iteration
%   x_predicted: vector of predicted states in the last iteration
%   x_Real: vector of measured vehicle states [x y theta v alpha w]
%   kin_states: number of kinematic states: 3
%   kin_inputs: number of kinematic control actions: 2
%   x_Ref: x reference (planner)
%   y_Ref: y reference (planner)
%   theta_Ref: theta reference (planner)
%   vel_Ref: velocity reference (planner)
%   omega_Ref: angular velocity reference (planner)
%   longitud_vect: length of vectors for simulation
%   car: vehicle object 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if KC==1   
    % Initializing vectors:
    theta_cos           = ones(1,Hp+1);
    theta_sin           = zeros(1,Hp+1);
    theta_err_cos       = ones(1,Hp);   
    U_OPT_VECTOR        = zeros(kin_inputs,Hp,longitud_vect);
    X_OPT_VECTOR        = zeros(kin_states,Hp,longitud_vect);
end



%%%% Constraints %%%%
% Control action constraints:
    % CA
        u_min_ini   = vehicle.u_min_ini;
        u_max_ini   = vehicle.u_max_ini;     
        u_min       = vehicle.u_min;
        u_max       = vehicle.u_max;       
    % derivative of CA     
        du_min_ini  = vehicle.du_min_ini;
        du_max_ini  = vehicle.du_max_ini;
        du_min      = vehicle.du_min;
        du_max      = vehicle.du_max;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %% Terminal Set LMIs    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %[ PP, WW ] = TerminalSet_Continuous_Knemtic_Cmputtion();
    %[ PP, WW ] = TerminalSet_Discrete_Knemtic_Cmputtion(Ts_Kcontrol);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Errors de verdad (no forecasted from the LPV model):
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
    x_err(1)    = (x_Ref(1)-x_Real(1))*cos(x_Real(3)) + (y_Ref(1)-x_Real(2))*sin(x_Real(3));
    x_err(2)    = -(x_Ref(1)-x_Real(1))*sin(x_Real(3)) + (y_Ref(1)-x_Real(2))*cos(x_Real(3));
    x_err(3)    = theta_Ref(1) - x_Real(3);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Moving Horizon Constraints:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if KC>1
        for j=1:Hp+1          
            theta_cos(j) = cos(theta_Ref(1));
            theta_sin(j) = sin(theta_Ref(1)); 
            if(j>1)
                theta_err_cos(j-1) = cos(x_predicted(3,1)); % cos (theta_ERROR forecasted)
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Computing PREDICTIONS of KIN-DYN LPV Vehicle Model. Note that
        % this A and B matrices are in continuous time due to the
        % discretization is carried out inside the controller object.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        [Ac_pred, Bc_pred, Bref] = KinemError_LPV_Model_predicted(KC,Hp,x_predicted(3,:),...
            u_predicted(2,:), vel_Ref );    
        for i=1:Hp
            Ac_pred(:,:,i) = Ac_pred(:,:,1);
            Bc_pred(:,:,i) = Bc_pred(:,:,1);
        end
    else
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Computing KIN-DYN LPV Vehicle Model for First iteration. Note that
        % this A and B matrices are in continuous time due to the
        % discretization is carried out inside the controller object.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        [Ac_pred, Bc_pred, Bref] = KinemError_LPV_Model(KC,Hp,x_err(3),...
            x_Real(6), vel_Ref(1) ); 
    end    
    
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Optimization stage:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     if(KC > car.t_ini)    
        inputs = {x_err', oldu, vel_Ref, omega_Ref,...
            theta_cos, theta_sin, u_min, u_max, du_min, du_max};    
%     else
%         inputs = {x_err', oldu, vel_Ref, omega_Ref,...
%             theta_cos, theta_sin, u_min_ini, u_max_ini,...
%             du_min_ini, du_max_ini};    
%     end
    
    [solutions,diagnostics] = controller{inputs};
    

    if diagnostics == 1
        u_opt = oldu;
        U_OPT_VECTOR(:,:,KC) = U_OPT_VECTOR(:,:,KC-1);
        X_OPT_VECTOR(:,:,KC) = X_OPT_VECTOR(:,:,KC-1);
    else
        U_OPT_VECTOR(:,:,KC) = solutions{1};
        u_opt = U_OPT_VECTOR(:,1,KC);
        X_OPT_VECTOR(:,:,KC) = solutions{2}; % Kinematic errors
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Predictions obtained from the solver:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x_predicted = X_OPT_VECTOR(:,:,KC);
    u_predicted = U_OPT_VECTOR(:,:,KC);            

    
        
        
end