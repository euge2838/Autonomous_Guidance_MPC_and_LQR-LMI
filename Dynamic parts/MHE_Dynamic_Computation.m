%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [XX_EST_VECTOR, Fric_Force, A_past, B_past] = MHE_Dynamic_Computation...
    ( Ts_Dcontrol, x_estimated, y_MHE, u_MHE, Kalman_Observer, Hp_obs, index_Dyn, ForceDist, Steer)
    
    A_past          = zeros(3,3,Hp_obs-1);
    B_past          = zeros(3,2,Hp_obs-1);

    C   = [1 0 0;
           0 0 1];
    m   = 683;
    E   = [-Ts_Dcontrol/m;
             0;
             0];
%     E   = [-1/m;
%              0;
%              0];
    OO  = pinv(C*E);
        
    if (index_Dyn > Hp_obs+1)    
        for i=1:Hp_obs-1
            [ A, B ] = A_OL_SYMPLIFIED_DYNAMIC_DISCRETE( x_estimated(1,i),...
                x_estimated(2,i), u_MHE(2,i), Ts_Dcontrol );
            A_past(:,:,i) = (eye(3)-E*OO*C)*A;
            B_past(:,:,i) = (eye(3)-E*OO*C)*B;
        end

        XXX = [x_estimated(1,1) x_estimated(2,1) x_estimated(3,1)]';

        [solutions,diagnostics] = Kalman_Observer{{XXX, y_MHE, u_MHE, A_past, B_past}};

        if diagnostics == 1
           error('The problem is infeasible');
        end

        XX_EST_VECTOR = solutions;  
      
        Fric_Force = pinv(C*E) * ( y_MHE(:,end) - C*...
                 (A*XX_EST_VECTOR(:,end-1) + B*u_MHE(:,end) ) ); %[ForceDist;Steer] ));%
             
    % Disturbance estimation:   
%         f_est(index_Dyn) = pinv(Ts_Dcontrol*C*E) * ( y_MHE(:,end) - ...
%             ( C*A*XX_EST_VECTOR(:,end-1) + C*B*[ForceDist;Steer] ) );            
%         disturbance_estimated = f_est(index_Dyn);
        
     
    %%
    else
        for i=1:Hp_obs-1
            [ A, B ] = A_OL_SYMPLIFIED_DYNAMIC_DISCRETE( x_estimated(1,1),...
                x_estimated(2,1), u_MHE(2,1), Ts_Dcontrol );
            A_past(:,:,i) = (eye(3)-E*OO*C)*A;
            B_past(:,:,i) = (eye(3)-E*OO*C)*B;
        end

        XXX = [x_estimated(1,1) x_estimated(2,1) x_estimated(3,1)]';

        [solutions,diagnostics] = Kalman_Observer{{XXX, y_MHE, u_MHE, A_past, B_past}};

        if diagnostics == 1
           error('The problem is infeasible');
        end
        
        XX_EST_VECTOR = solutions;
       
        Fric_Force = pinv(Ts_Dcontrol*C*E) * ( y_MHE(:,end) - C*( ...
            A*XX_EST_VECTOR(:,end-1) + B*u_MHE(:,end) ) );
%         disturbance_estimated = 0;
    end
end

% %         [ x_est, f_est ] = State_Estimation( x_est, y_for_Obs, ForceDist, Steer, C_obs,...
% %                             index_Dyn, index, L1, L2, L3, L4, L5, L6, L7, L8, Ts_Dcontrol);