%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eugenio Alcala Baselga
% Date: 02/06/2018
%
% Description: Vehicle control with MPC kinematic layer plus terminal
% state, dynamic LPV-LQR state feedback and dynamic Moving Horizon
% Estimator (MHE).
%
% Two layers: Kinematic and Dynamic.
% Upper layer: Kinematic MPC ensuring stability with a terminal constraint
% computing with LMIs. 
% Inner layer: DISCRETE LPV-LQR state feedback. Dynamic model with 6
% states.
% 
% When launching the script, first, it allows you to choose between the set
% of possible algorithms.
% There are 3 strategies:
%   1.- DISCRETE frozen-MPC control. This method does not update the system
%       matrices during the prediction stage.
%   2.- DISCRETE NL-MPC control. Non-linear version of this problem.
%   3.- DISCRETE references-MPC control. This technique update the system
%       matrices bu using the references provided by the traj. planner.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc

yalmip('clear');

n = input('Choose a kinematic control law. \n 1.- DISCRETE frozen-MPC control \n 2.- DISCRETE NL-MPC control \n 3.- DISCRETE References-MPC control \n');

switch n
    case 1
        CL = 1;
        frozen_based        = true;
        references_based    = false;        
    case 2
        CL = 2; 
    case 3
        CL = 1;  
        frozen_based        = false;
        references_based    = true;         
    otherwise
        disp('NO VALID INPUT')
end

% Constants:
RADTODEG = 57.2958;
DEGTORAD = 0.0175;

   
%% Sample time and Prediction Horizon
    % Dynamic control sample time:
    Ts_Dcontrol = automatic_dynamic_control.Ts; % 1 ms

        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%  --> KINEMATIC MPC DESIGN <--  %%%%%%%%%%%%%%%    
% Choose control law:
%   - State feedback LQR-LMI based control
%   - MPC control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
if CL == 1
%     Ts_Kcontrol = 0.05;
    Ts_Kcontrol = 0.1;
    % Compute the Terminal Weigh and Set to guarantee the MPC stability:
    [ PP, WW ] = TerminalSet_Discrete_Knemtic_Cmputtion(Ts_Kcontrol);    
    [controller, kin_states, kin_inputs, Hp] = MPC_Kinematic_Design(Ts_Kcontrol, PP, WW);
elseif CL == 2
    Ts_Kcontrol = 0.1;
    % Compute the Terminal Weigh and Set to guarantee the MPC stability:
    [ PP, WW ] = TerminalSet_Discrete_Knemtic_Cmputtion(Ts_Kcontrol);    
    [controller, kin_states, kin_inputs, Hp] = NLMPC_Kinematic_Design(Ts_Kcontrol, PP, WW);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%  --> DYNAMIC LPV STATE FEEDBACK CONTROL DESIGN <--  %%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
[ Kd ] = Dynamic_Controller_Discrete_Cmputtion(Ts_Dcontrol);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%  --> MOVING HORIZON LPV KALMAN ESTIMATOR DESIGN <--  %%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Hp_obs = 30;
[ Kalman_Observer, dyn_states, dyn_inputs ] = MHE_Dynamic_Design(Hp_obs);



%% Choose simulation option:
%     option  = 1; %Start from the beginning
%     option  = 2; %Start from an advanced position
%     option  = 3; %Start from the most advanced position
sim_option  = 1;

%% TRAJECTORY PLANNING
    PROPOSED_SIMULATION_LENGTH = 1312;

%         planner_data = load('Planner_References_50ms.txt');
    planner_data = load('Planner_References_100ms.txt');

    if sim_option==1
        offset  = 5;   % -> u_ini   = [670 0]';
        fin     = PROPOSED_SIMULATION_LENGTH;
    elseif sim_option==2
        offset  = 1670; % -> u_ini   = [1050 -0.02]'; 
        fin     = 2800;
    elseif sim_option==3
        offset  = 2320; % -> u_ini   = [768 0]';  
        fin     = 2800;
    end
    final   = fin+offset-1;
   

    x_d     = planner_data(1,offset:final);
    y_d     = planner_data(2,offset:final);
    theta_d = planner_data(3,offset:final);
    v_d     = planner_data(4,offset:final);
    alpha_d = planner_data(5,offset:final);
    w_d     = planner_data(6,offset:final);
    
    
        % Plot the Road
    for ii=1:length(x_d)
        Pose(:,ii)        = [x_d(ii); y_d(ii)];
        R                 = [cos(theta_d(ii)) sin(theta_d(ii)); -sin(theta_d(ii)) cos(theta_d(ii))];
        NewPose(:,ii)     = R*Pose(1:2,ii);
        RightLine(:,ii)   = NewPose(1:2,ii) + [0; 4.5];
        LeftLine(:,ii)    = NewPose(1:2,ii) - [0; 1.5];
        CenterLine(:,ii)  = NewPose(1:2,ii) + [0; 1.5];
        RightLine(:,ii)   = inv(R)*RightLine(1:2,ii);
        LeftLine(:,ii)    = inv(R)*LeftLine(1:2,ii);
        CenterLine(:,ii)    = inv(R)*CenterLine(1:2,ii);
    end
    

    
    sim_time_vec    = Ts_Kcontrol:Ts_Kcontrol:(Ts_Kcontrol*PROPOSED_SIMULATION_LENGTH);


%% Vehicle Initialization:
    car         = initialize(vehicle, x_d(1),...
        y_d(1),theta_d(1),v_d(1),w_d(1), sim_option);
    x_Real      = [car.x car.y car.theta car.V car.AlphaT car.Ang_vel]';  % Output of the real vehicle [x y theta v alpha w]
    u_ini       = [car.V car.Ang_vel]';
    Force       = car.Fxr;
    ForceDist   = Force;
    Steer       = car.delta;
    Sigma       = car.sigma;
    u_opt       = u_ini;
    oldu        = u_ini;

                   
%% Simulation time and vectors
                

%% Preallocating vectors and matrices
    longitud_vect   = length(sim_time_vec);
    VEL_ERR         = zeros(longitud_vect,1);
    OMEGA_ERR       = zeros(longitud_vect,1);
    Y_ERR           = zeros(longitud_vect,1);
    X_ERR           = zeros(longitud_vect,1);
    THETA_ERR       = zeros(longitud_vect,1);
    x_err           = zeros(3,1);
    X               = zeros(longitud_vect,1);
    Y               = zeros(longitud_vect,1);
    THETA           = zeros(longitud_vect,1);
    VEL             = zeros(longitud_vect,1);
    ALPHA           = zeros(longitud_vect,1);
    OMEGA           = zeros(longitud_vect,1);

    TOC_PLOT        = zeros(1,longitud_vect);
    PLOT_U_KIN      = zeros(kin_inputs,longitud_vect);
    DYNAMIC_CA      = zeros(3,longitud_vect);

    x_predicted     = zeros(kin_states,Hp);
    u_predicted     = zeros(kin_inputs,Hp);
    
    A_past          = zeros(3,3,Hp_obs-1);
    B_past          = zeros(3,2,Hp_obs-1);
      
    % Observer vectors:
    %XX_EST_VECTOR   = zeros(dyn_states,Hp_obs);
    WW_EST_VECTOR   = zeros(dyn_states,Hp_obs-1);
    y_for_Obs       = 0.001*ones(2,longitud_vect);      % Output measured from sensors [v - w ]      
    y_MHE           = 0.001*ones(3,Hp_obs-1);             
    u_for_Obs       = 0.001*ones(2,longitud_vect*100);      % Input applyied to the vehicle  
    u_for_Obs(:,1)  = [ForceDist; Steer];
    u_MHE           = 0.001*ones(2,Hp_obs-1);
    
    X_EST           = zeros(3,longitud_vect);
    F_EST           = zeros(1,longitud_vect);
    x_est           = 0.001*ones(3,longitud_vect);  
    OBS_ERROR       = zeros(1,longitud_vect); 
    v_wind          = zeros(1,longitud_vect); 
    Disturb_PLOT    = zeros(1,longitud_vect); 
    mu_friction     = zeros(1,longitud_vect); 
    v_disturbance   = zeros(1,longitud_vect);
    
    OBS_ERR_V           = zeros(1,longitud_vect);
    OBS_ERR_ALPHA       = zeros(1,longitud_vect);
    OBS_ERR_W           = zeros(1,longitud_vect);
    OBS_ERR_FRICTION    = zeros(1,longitud_vect);
    F_FRICTION          = zeros(1,longitud_vect);
    ALPHA_EST           = zeros(1,longitud_vect);
    
    IDENT_VARS      = zeros(longitud_vect, 8);
    
%% Loop constants
counter     = 0;
count_diag  = 0;
index_Dyn   = 2;


Cff_Fric_dry_asphalt    = 0.9;
Cff_Fric_dry_earth      = 0.68;
Cff_Fric_gravel         = 0.6;
Cff_Fric_wet_asphalt    = 0.6;
Cff_Fric_snow           = 0.2;
Cff_Fric_ice            = 0.1;


Frict_Coeff         = zeros(1,longitud_vect);
F_friction          = zeros(1,longitud_vect);
%% NOTE: Uncomment this if we wanna insert the disturbance case.

for i=1:length(Frict_Coeff)
    Frict_Coeff(i) = Cff_Fric_dry_asphalt + 0.002*sin(0.05*i);
end
for i=150:160
    Frict_Coeff(i) = Frict_Coeff(i) - 0.0221*(i-150);
end
for i=161:389
    Frict_Coeff(i) = Cff_Fric_dry_earth;
end
for i=390:400
    Frict_Coeff(i) = Cff_Fric_dry_earth + 0.0221*(i-390);
end
for i=700:701
    Frict_Coeff(i) = Cff_Fric_ice;
end
for i=1101:1111
    Frict_Coeff(i) = Frict_Coeff(i) - 0.03*(i-1101);
end
for i=1111:1190
    Frict_Coeff(i) = Cff_Fric_wet_asphalt;
end
for i=1190:1200
    Frict_Coeff(i) = Cff_Fric_wet_asphalt + 0.0285*(i-1190);
end

m       = 683;
g       = 9.81;      
for i=1:length(Frict_Coeff)
    F_friction(i) = Frict_Coeff(i)*m*g;
%     F_friction(i) = 0.65*m*g;
end

figure(99), plot(sim_time_vec,F_friction,'k'),grid on
xlabel('Time [s]')
ylabel('Friction force [N]')


tK = Ts_Dcontrol;
for KC=1:length(sim_time_vec)-Hp
    
    %clc
    tK          = tK + Ts_Kcontrol; % Time counter
    percentage  = (KC/length(sim_time_vec))*100;
    XX          = ['*** ',num2str(percentage), '%', ' ***'];
    disp(XX)

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Planner Window:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    vel_Ref     = v_d(KC:KC+Hp);
    omega_Ref   = w_d(KC:KC+Hp-1);
    x_Ref       = x_d(KC:KC+Hp-1);
    y_Ref       = y_d(KC:KC+Hp-1);
    theta_Ref   = theta_d(KC:KC+Hp);  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematic MPC Control:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    tic;
    if CL == 1
        [ u_opt, u_predicted, x_predicted, x_err ] = MPC_Kinematic_Computation...
            ( controller, Hp, KC, oldu, u_predicted, x_predicted, x_Real,...
            kin_states, kin_inputs, x_Ref, y_Ref, theta_Ref, vel_Ref, omega_Ref,...
            longitud_vect, car, frozen_based, references_based);   
        oldu = u_opt;
        PLOT_U_KIN(:,KC)= u_opt;
    elseif CL == 2
        [ u_opt, u_predicted, x_predicted, x_err ] = NLMPC_Kinematic_Computation...
            ( controller, Hp, KC, oldu, u_predicted, x_predicted, x_Real,...
            kin_states, kin_inputs, x_Ref, y_Ref, theta_Ref, vel_Ref, omega_Ref,...
            longitud_vect, car);   
        oldu = u_opt;
        PLOT_U_KIN(:,KC)= u_opt;        
    end
    TOC_PLOT(KC) = toc;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Dynamic LPV Control [DISCRETE]:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for tD=tK:Ts_Dcontrol:tK+Ts_Kcontrol

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %\\ Moving Horizon Dynamic Estimator (MHE):
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if (index_Dyn > Hp_obs+1)
            [ XX_EST_VECTOR, Fric_Force, A_D, B_D] = MHE_Dynamic_Computation( Ts_Dcontrol,...
                XX_EST_VECTOR, y_MHE, u_MHE, Kalman_Observer, Hp_obs, index_Dyn, ForceDist, Steer);
            x_est = XX_EST_VECTOR(:,end);
        elseif (index_Dyn > Hp_obs)    
            [ XX_EST_VECTOR, Fric_Force, A_D, B_D ] = MHE_Dynamic_Computation( Ts_Dcontrol,...
                [y_MHE(1,1) 0 y_MHE(2,1)]', y_MHE, u_MHE, Kalman_Observer, Hp_obs,...
                index_Dyn, ForceDist, Steer);
            x_est = XX_EST_VECTOR(:,end);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %\\ Dynamic Control (Inner loop):
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if (index_Dyn > Hp_obs+1)
            [ Force, Steer, Sigma, poles_checking_D] = DynamicControl_Discrete( Kd, x_Real,...
                [ForceDist; Sigma], u_opt(1), u_opt(2), Ts_Dcontrol, tD);

            ForceDist = Force;% + Fric_Force;
        else
            [ Force, Steer, Sigma, poles_checking_D] = DynamicControl_Discrete( Kd, x_Real,...
                [ForceDist; Sigma], u_opt(1), u_opt(2), Ts_Dcontrol, tD);                
            ForceDist = Force;
        end
        
        if (ForceDist < -2000)
            ForceDist = -2000;
        elseif (ForceDist > 10000)
            ForceDist = 10000;
        end  
        
        u_for_Obs(:,index_Dyn) = [ForceDist; Steer];

        % % % At this point we change the iteration time % % %
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %\\ Vehicle simulation:
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        T = tD:Ts_Dcontrol/2:tD+Ts_Dcontrol; 
        % [T,x] = ode45(@(t,x) dynamic_complex_model(t,x,[ ForceDist; Steer; Frict_Coeff(KC)]), T, x_Real);  
        [T,x] = ode45(@(t,x) dynamic_complex_model(t,x,[ ForceDist; Steer; 0.65]), T, x_Real);  
        x_Real = [x(end,1); x(end,2); x(end,3); x(end,4); x(end,5); x(end,6)];
        
        index_Dyn = index_Dyn + 1; 

        C = [1 0 0; 0 0 1];
        y_for_Obs(:,index_Dyn) = C*[x_Real(4); x_Real(5); x_Real(6)];
        
        if (index_Dyn > Hp_obs)
            y_MHE = y_for_Obs(:,index_Dyn-Hp_obs+2:index_Dyn);
            u_MHE = u_for_Obs(:,index_Dyn-Hp_obs+1:index_Dyn-1);
        end
        
    end % END DYNAMIC LOOP

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vectors for plotting:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    %% OBSERVER:  
    V_EST(KC)       = x_est(1);
    ALPHA_EST(KC)   = x_est(2);
    W_EST(KC)       = x_est(3);    
%     F_FRICTION(KC)  = Fric_Force;
    
    OBS_ERR_V(KC)       = x_Real(4)-x_est(1);
    OBS_ERR_ALPHA(KC)   = x_Real(5)-x_est(2);
    OBS_ERR_W(KC)       = x_Real(6)-x_est(3);    
    
    %% CONTROLLER VARIABLES:
    X_ERR(KC)       = x_err(1);
    Y_ERR(KC)       = x_err(2);
    THETA_ERR(KC)   = x_err(3);
    DYNAMIC_CA(:,KC)= [ForceDist; Force; Steer];    
    
    %% VEHICLE MEASUREMENTS:
    X(KC)           = x_Real(1);
    Y(KC)           = x_Real(2);
    if (x_Real(3) < -pi)
        THETA(KC)   = x_Real(3) + 2*pi;
    elseif(x_Real(3) > pi)
        THETA(KC)   = x_Real(3) - 2*pi;
    else
        THETA(KC)   = x_Real(3);
    end
    THETA(KC)       = x_Real(3);
    VEL(KC)         = x_Real(4);
    ALPHA(KC)       = x_Real(5);
    OMEGA(KC)       = x_Real(6);
    
    VEL_ERR(KC)     = v_d(KC)-VEL(KC);
    OMEGA_ERR(KC)   = w_d(KC)-OMEGA(KC);
           
    counter = counter + 1;
    if(counter > 50) 

        color = 'b'; %[y m c r g b w k]       
        figure(2)   
      subplot(5,1,1)
        hold on, plot(X_ERR(1:KC),color);
        ylabel('Long error [m]'), grid on
      subplot(5,1,2) 
        hold on, plot(Y_ERR(1:KC),color)
        ylabel('Lateral error [m]'), grid on
      subplot(5,1,3)
        hold on, plot(RADTODEG*THETA_ERR(1:KC),color)
        ylabel('Orientation error [º]'), grid on
      subplot(5,1,4)
        hold on, plot(3.6*VEL_ERR(1:KC),color)
        ylabel('Vel error [km/h]'), grid on
      subplot(5,1,5)
        hold on, plot(RADTODEG*OMEGA_ERR(1:KC),color)
        ylabel('Ang. Vel error [º/s]'), grid on
        drawnow
        counter = 0;
    end
    
end % END KINEMATIC LOOP

color = 'b';

figure(1)
subplot(2,1,1)
hold on, plot(sim_time_vec(1,1:KC),3.6*VEL(1:KC),color,'linewidth',0.8);
hold on, plot(sim_time_vec(1,1:KC),3.6*v_d(1:KC),'--r','linewidth',0.7)
ylabel('Linear velocity [Km/h]'), grid on
xlabel('Time [s]')
subplot(2,1,2)
hold on, plot(sim_time_vec(1,1:KC),RADTODEG*OMEGA(1:KC),color,'linewidth',0.8);
hold on, plot(sim_time_vec(1,1:KC),RADTODEG*w_d(1:KC),'--r','linewidth',0.7);
ylabel('Angular velocity [º/s]'), grid on
xlabel('Time [s]')

figure(2)
subplot(2,1,1)
hold on, plot(sim_time_vec(1,1:KC),DYNAMIC_CA(1,1:KC),'r','linewidth',0.8);     % ForceDist
hold on, plot(sim_time_vec(1,1:KC),DYNAMIC_CA(2,1:KC),color,'linewidth',0.8);   % Force
ylabel('Force [N]'), grid on  
subplot(2,1,2)
hold on, plot(sim_time_vec(1,1:KC),RADTODEG*DYNAMIC_CA(3,1:KC),color,'linewidth',0.8);
ylabel('Steering Angle [º]'), grid on

figure(3)   
subplot(5,1,1)
hold on, plot(sim_time_vec(1,1:KC),X_ERR(1:KC),color,'linewidth',0.8);
title('Longitudinal error [m]'), grid on
subplot(5,1,2) 
hold on, plot(sim_time_vec(1,1:KC),Y_ERR(1:KC),color,'linewidth',0.8);
title('Lateral error [m]'), grid on
subplot(5,1,3)
hold on, plot(sim_time_vec(1,1:KC),RADTODEG*THETA_ERR(1:KC),color,'linewidth',0.8);
title('Orientation error [º]'), grid on
subplot(5,1,4)
hold on, plot(sim_time_vec(1,1:KC),3.6*VEL_ERR(1:KC),color,'linewidth',0.8);
title('Lnear velocity error [km/h]'), grid on
subplot(5,1,5)
hold on, plot(sim_time_vec(1,1:KC),RADTODEG*OMEGA_ERR(1:KC),color,'linewidth',0.8);
title('Angular elocity error [º/s]'), grid on
        
figure(4)
hold on, 
plot(TOC_PLOT(1:KC),'black')
grid on
ylabel('Elapsed time [s]')
xlabel('Iterations')
Mean_ElapsedTime = (sum(TOC_PLOT(1:KC)))/length(TOC_PLOT(1:KC))

figure(5)
hold on, plot(sim_time_vec(1:KC),F_FRICTION(1:KC) + 0.5*9.81*683,'black')
hold on, plot(sim_time_vec(1:KC),F_friction(1:KC),'--r')
ylabel('Friction force [N]')
xlabel('Time [s]')
legend('Estimated friction force','Vehicle friction force')
grid on

figure(6)
plot(LeftLine(1,1:end),LeftLine(2,1:end),'k','LineWidth',1)
hold on, plot(RightLine(1,1:end), RightLine(2,1:end),'k','LineWidth',1)
hold on, plot(CenterLine(1,1:end),CenterLine(2,1:end), '--k','LineWidth',1)
hold on, plot(x_d,y_d, '--r')
hold on, plot(X(1:KC),Y(1:KC),'g')
hold on, plot(x_d(KC:end)-0.1,y_d(KC:end)-0.01, 'g')
grid on
xlabel('X [m]')
ylabel('Y [m]')

color = 'g'
figure(6)
subplot(2,1,1)
plot(LeftLine(1,1:end),LeftLine(2,1:end),'k','LineWidth',1)
hold on, plot(RightLine(1,1:end), RightLine(2,1:end),'k','LineWidth',1)
hold on, plot(CenterLine(1,1:end),CenterLine(2,1:end), '--k','LineWidth',1)
hold on, plot(x_d,y_d, '--r')
hold on, plot(X(1:KC),Y(1:KC),color)
hold on, plot(x_d(KC:end)-0.1,y_d(KC:end)-0.01, color)
grid on
xlabel('X [m]')
ylabel('Y [m]')
subplot(2,1,2)
plot(LeftLine(1,1:end),LeftLine(2,1:end),'k','LineWidth',1)
hold on, plot(RightLine(1,1:end), RightLine(2,1:end),'k','LineWidth',1)
hold on, plot(CenterLine(1,1:end),CenterLine(2,1:end), '--k','LineWidth',1)
hold on, plot(x_d,y_d, '--r')
hold on, plot(X(1:KC),Y(1:KC),color)
hold on, plot(x_d(KC:end)-0.1,y_d(KC:end)-0.01, color)
grid on
xlabel('X [m]')
ylabel('Y [m]')



%% MEAN SQUARED ERRORS

mse_xError      = sqrt((sum(X_ERR(1:end-50).^2))/length(X_ERR(1:end-50)))
mse_yError      = sqrt((sum(Y_ERR(1:end-50).^2))/length(Y_ERR(1:end-50)))
mse_thetaError  = sqrt((sum(THETA_ERR(1:end-50).^2))/length(THETA_ERR(1:end-50)))
mse_velError    = sqrt((sum(VEL_ERR(1:end-20).^2))/length(VEL_ERR(1:end-20)))
mse_omegaError  = sqrt((sum(OMEGA_ERR(1:end-20).^2))/length(OMEGA_ERR(1:end-20)))

% Hp
% mse_VEL_OBS         = sqrt((sum(OBS_ERR_V(1:end-50).^2))/length(OBS_ERR_V(1:end-50)))
% mse_ALPHA_OBS       = sqrt((sum(OBS_ERR_ALPHA(1:end-50).^2))/length(OBS_ERR_ALPHA(1:end-50)))
% mse_OMEGA_OBS       = sqrt((sum(OBS_ERR_W(1:end-50).^2))/length(OBS_ERR_W(1:end-50)))
% mse_FRICTION_OBS    = sqrt((sum(OBS_ERR_FRICTION(1:end-50).^2))/length(OBS_ERR_FRICTION(1:end-50)))




