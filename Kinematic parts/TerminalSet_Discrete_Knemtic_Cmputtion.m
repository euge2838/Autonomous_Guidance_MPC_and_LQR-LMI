%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ PP, WW ] = TerminalSet_Discrete_Knemtic_Cmputtion(Ts)
    %% Kinematic Controller computation:
    % System definition:
    n_inputs        = 2;
    n_states        = 3;
    n_sched_vars    = 3;    
    size            = length(automatic_kinematic_control.V_vec)*...
                    length(automatic_kinematic_control.W_vec)*...
                    length(automatic_kinematic_control.Theta_err_vec);     
    
    % Scheduling variables:
    % W, Vd, Thera_error
    index    = 1;
    A_OL     = zeros(3,3,size);   
    for l=1:length(automatic_kinematic_control.W_vec)
        W = automatic_kinematic_control.W_vec(l);
        for i=1:length(automatic_kinematic_control.V_vec) 
            V = automatic_kinematic_control.V_vec(i);
            for j=1:length(automatic_kinematic_control.Theta_err_vec) 
                THETA_ERR = automatic_kinematic_control.Theta_err_vec(j);
                [A_OL(:,:,index), B] = A_OL_KINEMATIC_3VARS_DISCRETE(V,W,THETA_ERR,Ts);
                POLOS = eig(A_OL(:,:,index));
                rank(ctrb(A_OL(:,:,index), B));
                index = index + 1;
            end
        end 
    end

    % CONTROLLER: LQR via H2 control
    Wlmi       = cell(n_inputs, n_states, 2^(n_sched_vars));
    LMI1        = cell(n_states+n_states+n_inputs, n_states+n_states+n_inputs, 2^(n_sched_vars));
    LMI2        = cell(n_states+n_states+n_inputs, n_states+n_states+n_inputs, 2^(n_sched_vars));
    LMI3        = cell(n_states+n_states+n_inputs, n_states+n_states+n_inputs, 2^(n_sched_vars));
    LMI4        = cell(n_states+n_states+n_inputs, n_states+n_states+n_inputs, 2^(n_sched_vars));
    LMI5        = cell(n_states+n_states, n_states+n_states, 2^(n_sched_vars));
    LMI6        = cell(n_inputs, n_inputs, 2^(n_sched_vars));
    
    Ylmi        = sdpvar(n_inputs,n_inputs);
    Pinv        = sdpvar(n_states,n_states); 
    Klmi        = zeros(n_inputs, n_states, 2^(n_sched_vars));
    CL_Poles    = zeros(n_states, 1, 2^(n_sched_vars));        
    Zlmi        = sdpvar(n_states,n_states);

    for i=1:length(Klmi)
        Wlmi{i} = sdpvar(n_inputs,n_states); 
    end
    
    % Feasibility Problem
    Qlmi       = diag([1 1.5 3]);
    Rlmi       = diag([1 3]); 
    
    u_max2lmi  = [18^2  0; 0 1.4^2];
    %y_max2lmi  = [0.3^2 0 0; 0 0.3^2 0; 0 0 0.1^2];

    for i=1:2^(n_sched_vars)
        LMI1{i}     = [Pinv     ( A_OL(:,:,i)*Pinv+B*Wlmi{i} )'   Pinv        Wlmi{i}' ;
                       A_OL(:,:,i)*Pinv+B*Wlmi{i}   Pinv         zeros(3,3)  zeros(3,2);
                       Pinv                       zeros(3,3)     inv(Qlmi)   zeros(3,2);
                       Wlmi{i}                    zeros(2,3)     zeros(2,3)  inv(Rlmi)];                       
        %         LMI3{i}     = [ u_max2lmi  Wlmi{i};
        %                         Wlmi{i}'     P]; 
        %         LMI4{i}     = [ y_max2lmi      A_OL(:,:,i)*Plmi + B*Wlmi{i};
        %                         (A_OL(:,:,i)*Plmi + B*Wlmi{i})'       Plmi];                         
    end            
    F = Pinv>=0;
    for i=1:2^(n_sched_vars)
        F = F + [LMI1{i}>=0];% + [LMI3{i}>=0] + [LMI4{i}>=0];
    end        
    ops = sdpsettings('warning',1,'verbose',1,'solver','sedumi','cachesolvers',1);
    optimize(F,[],ops);

    PP = inv(value(Pinv));
    for i=1:length(Wlmi)
        Klmi(:,:,i) = value(Wlmi{i})*PP; 
        CL_Poles(:,:,i) = eig(A_OL(:,:,i)+B*Klmi(:,:,i));
        if ( max(real(CL_Poles(:,:,i)))>1 || max(real(CL_Poles(:,:,i)))<-1 )
            disp('WARNING! Unstable system')
            i
        end
    end       
    %Klmi
    %CL_Poles
    
    % Terminal Region Problem
    for i=1:2^(n_sched_vars) 
        LMI5{i}     = [-Zlmi     Zlmi*(A_OL(:,:,i)+B*Klmi(:,:,i))';
                       (A_OL(:,:,i)+B*Klmi(:,:,i))*Zlmi     -Zlmi]; 
        LMI6{i}     = Klmi(:,:,i)*Zlmi*Klmi(:,:,i)' - u_max2lmi;
    end  
    for i=1:2^(n_sched_vars)
        if i==1
            F2 = [LMI5{i}<=0] + [LMI6{i}<=0];
        else
            F2 = F2 + [LMI5{i}<=0] + [LMI6{i}<=0];
        end
    end        
    optimize(F2,-trace(Zlmi),ops);
    WW = inv(value(Zlmi));
    
%     xx = sdpvar(3,1);
%     plot(xx'*WW*xx <= 1,[],'r');
%     xlabel('X error'),ylabel('Y error'),zlabel('\theta error')
%     grid on


end