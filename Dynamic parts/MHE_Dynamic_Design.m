%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Kalman_Observer, dyn_states, dyn_inputs] = MHE_Dynamic_Design(N)

    %% Solver options:
    options = sdpsettings('solver','gurobi');
    %options = sdpsettings('solver','sedumi');
    %options = sdpsettings(options,'verbose',1);
    %options = sdpsettings(options,'cachesolvers',1);
    %options = sdpsettings(options,'warning',0);
    %options.gurobi;


    dyn_states  = 3;
    dyn_inputs  = 2;

    Q = [10  0  0;
         0  10  0;
         0  0  2];
    R = [1/30 0; 
         0 1/30];
    P = 2*eye(3);
    
    uu          = sdpvar(dyn_inputs,N-1,'full');
    xest        = sdpvar(dyn_states,N,'full'); 
    ym          = sdpvar(dyn_states-1,N-1,'full');
    xxinitial   = sdpvar(dyn_states,1);
    A_D         = sdpvar(dyn_states,dyn_states,N-1,'full');
    B_D         = sdpvar(dyn_states,dyn_inputs,N-1,'full');
    C_D         = [1 0 0; 0 0 1];   
    m           = 683;
    Ts          = 0.001;
    E           = [-Ts/m;
                     0;
                     0];
    OO          = pinv(C_D*E);
    
    constraints = [];
    objective   = (xest(:,1)-xxinitial(:,1))'*P*(xest(:,1)-xxinitial(:,1));
    for k = 1:N-1
        objective  = objective+...
                 (xest(:,k+1)-A_D(:,:,k)*xest(:,k)-B_D(:,:,k)*uu(:,k) - E*OO*ym(:,k) )'...
                 * Q *...
                 (xest(:,k+1)-A_D(:,:,k)*xest(:,k)-B_D(:,:,k)*uu(:,k) - E*OO*ym(:,k) )+...
                 (ym(:,k) - C_D*xest(:,k))' * R * (ym(:,k) - C_D*xest(:,k));
    end
        
%     for k = 1:N-1
%         objective  = objective+...
%                  (xest(:,k+1)-A_D(:,:,k)*xest(:,k)-B_D(:,:,k)*uu(:,k))'*Q*...
%                  (xest(:,k+1)-A_D(:,:,k)*xest(:,k)-B_D(:,:,k)*uu(:,k)) +...
%                  (ym(:,k) - C_D*xest(:,k))'*R*(ym(:,k) - C_D*xest(:,k));
%     end
    
    parameters_in   ={ xxinitial, ym, uu, A_D, B_D };
    solutions_out   = { xest };
    Kalman_Observer = optimizer(constraints, objective, options, parameters_in, solutions_out);

end

