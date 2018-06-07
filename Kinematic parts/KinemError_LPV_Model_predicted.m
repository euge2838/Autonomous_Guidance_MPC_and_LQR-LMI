%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Ac, Bc, Bref ] = KinemError_LPV_Model_predicted(KC,Hp,...
    theta_error_predicted, omega_Ref, vel_Ref )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 15/11/2017
% Description:
% This script computes the future LPV matrices from predicted states in the
% last optimization. This set of matrices will be used for refreshing the
% LPV matrices within the optimization loop.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:Hp
    %delta = atan(1.036*omega_predicted(i)/vel_predicted(i));
    [ Ac(:,:,i), Bc(:,:,i), Bref ] = KinemError_LPV_Model...
        (KC,Hp,theta_error_predicted(i), omega_Ref(i), vel_Ref(i));     
end

