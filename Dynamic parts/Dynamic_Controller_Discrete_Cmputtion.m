%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Klmi ] = Dynamic_Controller_Discrete_Cmputtion(Ts)
    %% Kinematic Controller computation:
    % System definition:
    n_inputs        = 2;
    n_states        = 6;
    n_sched_vars    = 3;    
    size            = length(automatic_dynamic_control.V_vec)*...
                length(automatic_dynamic_control.Steer_vec)*...
                length(automatic_dynamic_control.Alpha_vec);     
    
    % Scheduling variables:
    % V, Delta, Alpha
    index    = 1;
    A_OL     = zeros(n_states,n_states,size);   
    for l=1:length(automatic_dynamic_control.Steer_vec)
        Steer = automatic_dynamic_control.Steer_vec(l);
        for i=1:length(automatic_dynamic_control.V_vec) 
            V = automatic_dynamic_control.V_vec(i);
            for j=1:length(automatic_dynamic_control.Alpha_vec) 
                Alpha = automatic_dynamic_control.Alpha_vec(j);
                [A_OL(:,:,index), B] = A_OL_DYNAMIC_DISCRETE(V,Alpha,0,Steer,Ts);
                POLOS = eig(A_OL(:,:,index));
                rank(ctrb(A_OL(:,:,index), B));
                index = index + 1;
            end
        end 
    end

    % CONTROLLER: LQR via H2 control
    Wlmi        = cell(n_inputs, n_states, 2^(n_sched_vars));
    LMI1        = cell(n_states+n_states+n_states+n_inputs, n_states+n_states+n_states+n_inputs, 2^(n_sched_vars));
    Y           = sdpvar(n_states,n_states); 
    Klmi        = zeros(n_inputs, n_states, 2^(n_sched_vars));
    CL_Poles    = zeros(n_states, 1, 2^(n_sched_vars));  

    for i=1:2^(n_sched_vars)
        Wlmi{i} = sdpvar(n_inputs,n_states); 
    end

%     Q       = diag([0.3 0.05 0.1 0.05 0.2 0.3]) * (1/6)*diag([0.003 156 1 1.5625e-08 5.253 1]);          
%     R       = diag([0.99 0.01])         * (1/2)*diag([1.5625e-08 5.253]);

%     Q       = diag([1000 0.1 0.1 0.1 100000 90000]);     % IET: muy bueno     
%     R       = diag([0.0003 10]);
    
    Q       = diag([2500 0.1 0.1 0.1 100000 90000]);          
    R       = diag([0.001 9]);
    
    % IET configuration. It does not work due to right now the controller
    % is in discrete time.
%     Q       = diag([0.05 0.01 0.01 0.01 100000 90000]);  
%     R       = diag([0.01 10]);  



    for i=1:2^(n_sched_vars)                      
        LMI1{i}     = [-Y     Y*A_OL(:,:,i)'+Wlmi{i}'*B'   Y*(Q^0.5)'         Wlmi{i}';
                       A_OL(:,:,i)*Y+B*Wlmi{i}     -Y         zeros(n_states,n_states)     zeros(n_states,2);
                       (Q^0.5)*Y                zeros(n_states,n_states)     -eye(n_states)       zeros(n_states,2);
                       Wlmi{i}                  zeros(2,n_states)     zeros(2,n_states)   -inv(R)];
    end            

    F = Y>=0;
    for i=1:2^(n_sched_vars)
        F = F + [LMI1{i}<=0];
    end        
    ops = sdpsettings('warning',1,'verbose',1,'solver','sedumi','cachesolvers',1);
    optimize(F,[],ops);

    for i=1:length(Wlmi)
        Klmi(:,:,i) = value(Wlmi{i})*inv(value(Y)); 
        CL_Poles(:,:,i) = eig(A_OL(:,:,i)+B*Klmi(:,:,i));
        CL_Poles(:,:,i);
        if ( max(real(CL_Poles(:,:,i)))>1 || max(real(CL_Poles(:,:,i)))<-1 )
            disp('WARNING! Unstable system')
            i
        end
    end       
    %Klmi

end

