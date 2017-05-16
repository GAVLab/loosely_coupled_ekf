clear,clc,%close all

load('../Carsim/carsim_gps_ins_data')
clear prm


%% --- Parameters
% User parameters
prm.POSITION_UPDATE = true;
prm.VELOCITY_UPDATE = true;
% Time
dt = mean(diff(imu.time));  
% Accel
prm.std_f               = 2e-03; % [(m/s^2)/sqrt(Hz)]
prm.std_b_f             = 7e-05; % [(m/s^2)/sqrt(s)]
prm.std_bias_acc_i      = 7e-04; % [m/s^2]
% Gyro
prm.std_g               = 2e-05; % [rad/s/sqrt(Hz)]
prm.std_b_g             = 4e-06; % [rad/s/sqrt(s)] 
prm.std_bias_gyr_i      = 5e-06; % [rad/s]
% GPS
prm.std_pos = 2;                % Standard deviation of GPS position [m]
prm.std_vel = 0.01;             % Standard deviation of GPS velocity [m/s]
prm.gps_dt = 1.0;               % Sample time of the GPS [s]
prm.ox = -(30.875+2.5)*0.0254;  % lever arm in the x direction [m]
prm.oy = -38.125*0.0254;        % lever arm in the y direction [m]
prm.oz = -39.5*0.0254;          % lever arm in the z direction [m]
% Initial estimate
prm.std_pos_i = prm.std_pos;
prm.std_vel_i = prm.std_vel;
prm.std_att_i = 0.01;
prm.pos_i = gps.pos(:,1);
prm.vel_i = gps.vel(:,1);
prm.att_i = truth.att(:,1) + prm.std_att_i*randn(3,1);
prm.bias_i = zeros(6,1);
% Feedback epochs
feedback_idx = (round(prm.gps_dt/dt):round(prm.gps_dt/dt):length(imu.time)) + 1; % feedback after measurement
prm.feedbackBool = zeros(size(imu.time));
prm.feedbackBool(feedback_idx) = 1;




est = gps_error_state( imu , gps , prm );


figure
plot(est.pos(2,:),est.pos(1,:)),hold on,grid on,axis equal