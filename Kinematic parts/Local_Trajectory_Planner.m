%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ x_d, y_d, phi_d, v_d, w_d] = Local_Trajectory_Planner( Ts_control, data_file )
    v_d     = data_file(:,1);
    av_d    = data_file(:,2);
    w_d     = data_file(:,3);
    aw_d    = data_file(:,4);

    x_d(1) = 0;
    y_d(1) = 0;
    phi_d(1) = 0;
    for i=2:length(v_d)
        phi_d(i)= limitare_unghi( phi_d(i-1) + w_d(i)*Ts_control);
        x_d(i)  = v_d(i) * cos(phi_d(i))*Ts_control + x_d(i-1);
        y_d(i)  = v_d(i) * sin(phi_d(i))*Ts_control + y_d(i-1);
    end
end

