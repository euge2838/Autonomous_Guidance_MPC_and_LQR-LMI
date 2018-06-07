%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function errors_pos_ex = errors_Lyapunov(input)

x_d = input(1);
y_d = input(2);
phi_d = input(3);

x_r = input(4);
y_r = input(5);
phi_r = input(6);

% Lyapunov error computation
e_x = (x_d-x_r)*cos(phi_d)  + (y_d-y_r)*sin(phi_d);
e_y = -(x_d-x_r)*sin(phi_d) + (y_d-y_r)*cos(phi_d);
e_phi = limitare_unghi(phi_d - phi_r);

% SMC error computation
% Lh = -2.5;
% e_x = (x_d-x_r + Lh*cos(phi_r))*cos(phi_d) + (y_d-y_r + Lh*sin(phi_r))*sin(phi_d);
% e_y = -(x_d-x_r + Lh*cos(phi_r))*sin(phi_d) + (y_d-y_r + Lh*sin(phi_r))*cos(phi_d);
% e_phi = limitare_unghi(phi_d - phi_r);

errors_pos_ex = [e_x; e_y; e_phi];