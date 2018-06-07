%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ controller, kin_states, kin_inputs, Hp ] = NLMPC_Kinematic_Design( Ts_Kcontrol, PP, WW )
% This function computes the terminal set for ensuring stability in the
% kinematic Model Predictive Control.

    Hp = 40;
    Q   = diag([0.9 0.025 0.075]) * (1/3)*diag([4 4 400]);
    R   = diag([0.001 0.999])   * (1/2)*diag([1 11]); 
    
%% SDPVARS:    
    kin_states              = 3;
    kin_inputs              = 2;
    
    u_OPTIM             = sdpvar(kin_inputs,1,Hp,'full'); 
    du_OPTIM            = sdpvar(kin_inputs,1,Hp,'full'); 
    u_min_OPTIM         = sdpvar(kin_inputs,1);
    u_max_OPTIM         = sdpvar(kin_inputs,1);
    du_min_OPTIM        = sdpvar(kin_inputs,1);
    du_max_OPTIM        = sdpvar(kin_inputs,1);
    pastu_OPTIM         = sdpvar(kin_inputs,1);
    x_OPTIM             = sdpvar(kin_states,1,Hp+1,'full'); 
    errors_OPTIM        = sdpvar(kin_states,1,Hp+1,'full'); 
    omega_Ref_OPTIM     = sdpvar(1,Hp);
    theta_cos_OPTIM     = sdpvar(1,Hp+1);
    theta_sin_OPTIM     = sdpvar(1,Hp+1);
    vel_Ref_OPTIM       = sdpvar(1,Hp+1);

    
%% Solver options:
    options = sdpsettings('solver','fmincon');
    options = sdpsettings(options,'verbose',1);
    options = sdpsettings(options,'cachesolvers',1);
    %options = sdpsettings(options,'warning',0);
    options.gurobi;

%% Building the object controller:
    objective = 0;
    for K = 1:Hp
        if K==1
            constraints = du_OPTIM(:,:,K) == u_OPTIM(:,:,K) - pastu_OPTIM;
        else
            constraints = [constraints; du_OPTIM(:,:,K) == u_OPTIM(:,:,K) - u_OPTIM(:,:,K-1)];
        end       
        objective   = objective + errors_OPTIM(:,1,K)'*Q*errors_OPTIM(:,1,K) + du_OPTIM(:,:,K)'*R*du_OPTIM(:,:,K);

        constraints = [constraints; 
                    x_OPTIM(1,:,K+1) == x_OPTIM(1,:,K) + ( u_OPTIM(2,:,K)*x_OPTIM(2,:,K)+vel_Ref_OPTIM(K)*cos(x_OPTIM(3,:,K))-u_OPTIM(1,:,K) ) * Ts_Kcontrol;
                    
                    x_OPTIM(2,:,K+1) == x_OPTIM(2,:,K) + ( -u_OPTIM(2,:,K)*x_OPTIM(1,:,K)+vel_Ref_OPTIM(K)*sin(x_OPTIM(3,:,K)) ) * Ts_Kcontrol;
                    
                    x_OPTIM(3,:,K+1) == x_OPTIM(3,:,K) + ( omega_Ref_OPTIM(K)-u_OPTIM(2,:,K) ) * Ts_Kcontrol;
                    
                    errors_OPTIM(1,1,K) == x_OPTIM(1,:,K) *theta_cos_OPTIM(K) + x_OPTIM(2,:,K) *theta_sin_OPTIM(K);
                    errors_OPTIM(2,1,K) == -x_OPTIM(1,:,K) *theta_sin_OPTIM(K) + x_OPTIM(2,:,K) *theta_cos_OPTIM(K);
                    errors_OPTIM(3,1,K) == x_OPTIM(3,:,K);
                    du_min_OPTIM <= du_OPTIM(:,:,K) <= du_max_OPTIM;
                    u_min_OPTIM <= u_OPTIM(:,:,K) <= u_max_OPTIM;
                    ];                            
    end
    objective       = objective + errors_OPTIM(:,1,K+1)'*PP*errors_OPTIM(:,1,K+1);

    
    constraints     = [constraints;
                    errors_OPTIM(1,1,K+1) == x_OPTIM(1,:,K+1) *theta_cos_OPTIM(K+1) + x_OPTIM(2,:,K+1) *theta_sin_OPTIM(K+1);
                    errors_OPTIM(2,1,K+1) == - x_OPTIM(1,:,K+1) *theta_sin_OPTIM(K+1) + x_OPTIM(2,:,K+1) *theta_cos_OPTIM(K+1);
                    errors_OPTIM(3,1,K+1) == x_OPTIM(3,:,K+1);
                    errors_OPTIM(:,1,K+1)'*WW*errors_OPTIM(:,1,K+1) <= 1 ];

    parameters_in   = {x_OPTIM(:,:,1), pastu_OPTIM, vel_Ref_OPTIM, omega_Ref_OPTIM,...
                    theta_cos_OPTIM, theta_sin_OPTIM,...
                    u_min_OPTIM, u_max_OPTIM, du_min_OPTIM,...
                    du_max_OPTIM};

    solutions_out   = {u_OPTIM, x_OPTIM(:,:,2:end)}; 

    controller      = optimizer(constraints, objective, options, parameters_in, solutions_out); 




end

