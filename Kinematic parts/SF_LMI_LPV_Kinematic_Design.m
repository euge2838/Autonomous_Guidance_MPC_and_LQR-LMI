%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Kc ] = SF_LMI_LPV_Kinematic_Design()
%% Kinematic loop
        %% Vehicle variable bounds
        index       = 1;
        A_large     = zeros(3,3,length(automatic_kinematic_control.V_vec)*length(automatic_kinematic_control.W_vec)*length(automatic_kinematic_control.Theta_err_vec));   
        for l=1:length(automatic_kinematic_control.W_vec)
            W = automatic_kinematic_control.W_vec(l);
            for i=1:length(automatic_kinematic_control.V_vec) 
                Vd = automatic_kinematic_control.V_vec(i);
                for j=1:length(automatic_kinematic_control.Theta_err_vec) 
                    THETA_ERR = automatic_kinematic_control.Theta_err_vec(j);
                    [A_large(:,:,index), B_large] = A_OL_KINEMATIC_3VARS(Vd,W,THETA_ERR);
                    POLOS = eig(A_large(:,:,index));
                    rank(ctrb(A_large(:,:,index), B_large));
                    index = index + 1;
                end
            end 
        end

        %% CONTROLLER: LQR via H2 control
        P       = sdpvar(3,3);
        Y       = sdpvar(2,2);
        W001    = sdpvar(2,3);
        W002    = sdpvar(2,3);
        W003    = sdpvar(2,3);
        W004    = sdpvar(2,3);
        W005    = sdpvar(2,3);
        W006    = sdpvar(2,3);
        W007    = sdpvar(2,3);
        W008    = sdpvar(2,3);
        gamma   = automatic_kinematic_control.gamma;
        Q       = automatic_kinematic_control.Q;
        R       = automatic_kinematic_control.R;
        alpha   = 3;
        beta    = 0.1;
        
        H11     = A_large(:,:,1)*P + B_large*W001 + (A_large(:,:,1)*P + B_large*W001)' + 2*(alpha)*P;
        H12     = [-Y    (R^0.5)*W001; ((R^0.5)*W001)'   -P];
        H13     = trace((Q^0.5)*P*(Q^0.5)') + trace(Y);
        H14     = A_large(:,:,1)*P + B_large*W001 + (A_large(:,:,1)*P + B_large*W001)' + 2*(beta)*P;

        H21     = A_large(:,:,2)*P + B_large*W002 + (A_large(:,:,2)*P + B_large*W002)' + 2*(alpha)*P;
        H22     = [-Y   (R^0.5)*W002; ((R^0.5)*W002)'  -P];
        H23     = A_large(:,:,2)*P + B_large*W002 + (A_large(:,:,2)*P + B_large*W002)' + 2*(beta)*P;

        H31     = A_large(:,:,3)*P + B_large*W003 + (A_large(:,:,3)*P + B_large*W003)' + 2*(alpha)*P;
        H32     = [-Y   (R^0.5)*W003; ((R^0.5)*W003)'  -P];
        H33     = A_large(:,:,3)*P + B_large*W003 + (A_large(:,:,3)*P + B_large*W003)' + 2*(beta)*P;

        H41     = A_large(:,:,4)*P + B_large*W004 + (A_large(:,:,4)*P + B_large*W004)' + 2*(alpha)*P;
        H42     = [-Y   (R^0.5)*W004; ((R^0.5)*W004)'  -P];
        H43     = A_large(:,:,4)*P + B_large*W004 + (A_large(:,:,4)*P + B_large*W004)' + 2*(beta)*P;

        H51     = A_large(:,:,5)*P + B_large*W005 + (A_large(:,:,5)*P + B_large*W005)' + 2*(alpha)*P;
        H52     = [-Y   (R^0.5)*W005; ((R^0.5)*W005)'  -P];
        H53     = A_large(:,:,5)*P + B_large*W005 + (A_large(:,:,5)*P + B_large*W005)' + 2*(beta)*P;

        H61     = A_large(:,:,6)*P + B_large*W006 + (A_large(:,:,6)*P + B_large*W006)' + 2*(alpha)*P;
        H62     = [-Y   (R^0.5)*W006; ((R^0.5)*W006)'  -P];
        H63     = A_large(:,:,6)*P + B_large*W006 + (A_large(:,:,6)*P + B_large*W006)' + 2*(beta)*P;

        H71     = A_large(:,:,7)*P + B_large*W007 + (A_large(:,:,7)*P + B_large*W007)' + 2*(alpha)*P;
        H72     = [-Y   (R^0.5)*W007; ((R^0.5)*W007)'  -P];
        H73     = A_large(:,:,7)*P + B_large*W007 + (A_large(:,:,7)*P + B_large*W007)' + 2*(beta)*P;

        H81     = A_large(:,:,8)*P + B_large*W008 + (A_large(:,:,8)*P + B_large*W008)' + 2*(alpha)*P;
        H82     = [-Y   (R^0.5)*W008; ((R^0.5)*W008)'  -P];
        H83     = A_large(:,:,8)*P + B_large*W008 + (A_large(:,:,8)*P + B_large*W008)' + 2*(beta)*P;

        
        F       = [P>=0]+[-H11<0]+[H12<0]+[H13<gamma]+[H14<0]+[-H21<0]+[H22<0]+[H23<0]+[-H31<0]+...
                [H32<0]+[H33<0]+[-H41<0]+[H42<0]+[H43<0]...
                +[-H51<0]+[H52<0]+[H53<0]+[-H61<0]+[H62<0]+[H63<0]+[-H71<0]+[H72<0]+[H73<0]+[-H81<0]+[H82<0]+[H83<0];
        
            
        ops     = sdpsettings('solver','sedumi');
%         ops     = sdpsettings('solver','gurobi');
        [opt_info]  = optimize(F,[],ops);

        Pfeasible   = value (P);
        P_eigs      = eig(Pfeasible);
        %%% Las K's obtenidas mediante LMI's son en la forma -K por tanto: u = kx
        Kc(:,:,1)    = value(W001)*inv(Pfeasible);
        Kc(:,:,2)    = value(W002)*inv(Pfeasible);
        Kc(:,:,3)    = value(W003)*inv(Pfeasible);
        Kc(:,:,4)    = value(W004)*inv(Pfeasible);
        Kc(:,:,5)    = value(W005)*inv(Pfeasible);
        Kc(:,:,6)    = value(W006)*inv(Pfeasible);
        Kc(:,:,7)    = value(W007)*inv(Pfeasible);
        Kc(:,:,8)    = value(W008)*inv(Pfeasible);
        
%         polos1_k       = eig(A_large(:,:,1)+B_large*Kc(:,:,1));
%         polos2_k       = eig(A_large(:,:,2)+B_large*Kc(:,:,2));
%         polos3_k       = eig(A_large(:,:,3)+B_large*Kc(:,:,3));
%         polos4_k       = eig(A_large(:,:,4)+B_large*Kc(:,:,4));
%         polos5_k       = eig(A_large(:,:,5)+B_large*Kc(:,:,5));
%         polos6_k       = eig(A_large(:,:,6)+B_large*Kc(:,:,6));
%         polos7_k       = eig(A_large(:,:,7)+B_large*Kc(:,:,7));
%         polos8_k       = eig(A_large(:,:,8)+B_large*Kc(:,:,8));

end

