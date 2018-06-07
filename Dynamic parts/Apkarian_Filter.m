%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Force, sigma] = Apkarian_Filter(u_new, controller_inputs, Ts)
    filter_Gain = 10;
    dyn_y       = -filter_Gain * controller_inputs + filter_Gain * u_new;
    controller_inputs  = controller_inputs + dyn_y*Ts;
    
    Force = controller_inputs(1);
    sigma = controller_inputs(2);

end

