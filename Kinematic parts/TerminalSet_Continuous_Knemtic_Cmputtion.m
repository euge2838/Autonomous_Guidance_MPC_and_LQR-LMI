%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ PP, WW ] = TerminalSet_Continuous_Knemtic_Cmputtion()
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
                [A_OL(:,:,index), B] = A_OL_KINEMATIC_3VARS(V,W,THETA_ERR);
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
    LMI6        = cell(1, 1, 2^(n_sched_vars));
    
    Ylmi        = sdpvar(n_inputs,n_inputs);
    Plmi        = sdpvar(n_states,n_states); 
    Klmi        = zeros(n_inputs, n_states, 2^(n_sched_vars));
    CL_Poles    = zeros(n_states, 1, 2^(n_sched_vars));        
    Zlmi        = sdpvar(n_states,n_states);

    for i=1:length(Klmi)
        Wlmi{i} = sdpvar(n_inputs,n_states); 
    end
    
    % Feasibility Problem
    gammalmi   = 0.01;
    Qlmi       = diag([1 1.5 3]);
    Rlmi       = diag([1 3]); 
    alphalmi   = 0.01;
    u_max2lmi  = [18^2  0; 0 1.4^2];
    y_max2lmi  = [0.3^2 0 0; 0 0.3^2 0; 0 0 0.1^2];

    LMI    = trace((Qlmi^0.5)*Plmi*(Qlmi^0.5)') + trace(Ylmi);
    for i=1:2^(n_sched_vars)
        LMI1{i}     = A_OL(:,:,i)*Plmi + B*Wlmi{i} + (A_OL(:,:,i)*Plmi + B*Wlmi{i})' + 2*(alphalmi)*Plmi;
        LMI2{i}     = [-Ylmi    (Rlmi^0.5)*Wlmi{i}; ((Rlmi^0.5)*Wlmi{i})'   -Plmi];
        LMI3{i}     = [ u_max2lmi  Wlmi{i};
                        Wlmi{i}'     Plmi]; 
        LMI4{i}     = [ y_max2lmi      A_OL(:,:,i)*Plmi + B*Wlmi{i};
                        (A_OL(:,:,i)*Plmi + B*Wlmi{i})'       Plmi];                         
    end            
    F = Plmi>=0;
    F = F + [LMI<=gammalmi];
    for i=1:2^(n_sched_vars)
        F = F + [LMI1{i}<=0] + [LMI2{i}<=0] + [LMI3{i}>=0] + [LMI4{i}>=0];
    end        
    ops = sdpsettings('warning',1,'verbose',1,'solver','sedumi','cachesolvers',1);
    optimize(F,[],ops);

    PP = value(Plmi);
    for i=1:length(Wlmi)
        Klmi(:,:,i) = value(Wlmi{i})*inv(value(Plmi)); 
        CL_Poles(:,:,i) = eig(A_OL(:,:,i)+B*Klmi(:,:,i));
        if ( max(real(CL_Poles(:,:,i)))>0 )
            disp('WARNING! Unstable system')
            i
        end
    end       
    %Klmi
    %CL_Poles
    
    % Terminal Region Problem
    for i=1:2^(n_sched_vars) 
        LMI5{i}     = (A_OL(:,:,i)+B*Klmi(:,:,i))*Zlmi + Zlmi*(A_OL(:,:,i)+B*Klmi(:,:,i))';                      
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
% %     hold on, plot(xx'*PP*xx <= 1,[],'b');
%     xlabel('X error'),ylabel('Y error'),zlabel('\theta error')
%     grid on


end

