%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Error State GPS (Loosely Coupled)
%
%   Author: Dan Pierce
%   Date: 2016.11.13
%
%   Description: An error state EKF is used to estimate position, velocity, 
%       attitude, and IMU bias errors with updates of position and velocity
%       from GPS.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function est = gps_error_state( imu , gps , prms )

I = eye(3);

%% -------- Time
time = imu.time;
dt = mean(diff(time));

%% -------- Inputs
u = [imu.zAccelX;imu.zAccelY;imu.zAccelZ;imu.zGyroX;imu.zGyroY;imu.zGyroZ];

%% -------- Noise Properties
% Process noise covariance matrix
Q = diag([prms.std_f^2,prms.std_f^2,prms.std_f^2, ...
          prms.std_g^2,prms.std_g^2,prms.std_g^2, ...
          prms.std_b_f^2,prms.std_b_f^2,prms.std_b_f^2, ...
          prms.std_b_g^2,prms.std_b_g^2,prms.std_b_g^2]);
      
R1 = diag([prms.std_pos^2,prms.std_pos^2,prms.std_pos^2]);
R2 = diag([prms.std_vel^2,prms.std_vel^2,prms.std_vel^2]);

%% -------- Initialize
pos = prms.pos_i;           % initial position
vel = prms.vel_i;           % initial velocity (assumed still on init)
att = prms.att_i;           % initial attitude solution
Cbn = RotationMatrix(att);    % initial rotation matrix (from body to nav frame)

x = [zeros(9,1);prms.bias_i];            % error state + bias estimate

% State estimate covariance matrix
P = diag([prms.std_pos_i^2,prms.std_pos_i^2,prms.std_pos_i^2, ...
          prms.std_vel_i^2,prms.std_vel_i^2,prms.std_vel_i^2, ...
          prms.std_att_i^2,prms.std_att_i^2,prms.std_att_i^2, ...
          prms.std_bias_acc_i^2,prms.std_bias_acc_i^2,prms.std_bias_acc_i^2, ...
          prms.std_bias_gyr_i^2,prms.std_bias_gyr_i^2,prms.std_bias_gyr_i^2]);

%% -------- Preallocate
est_save = nan(15,length(time)); % saves the PVA solution + bias estimates
var_save = nan(15,length(time)); % saves the diagonal of P
err_save = nan(9,length(time)); % saves the error estimate

update_bool = zeros(1,length(time));

resid_save = zeros(3*prms.POSITION_UPDATE + 3*prms.VELOCITY_UPDATE,length(gps.time));

err_save(:,1) = x(1:9);

gpsk = 2;
gps.time(end+1)=nan;
%% -------- Process
for k = 2:length(time)
    
    acc = u(1:3,k-1); % raw accel signal
    omega = u(4:6,k-1); % raw gyro signal
    
    acc_ = acc - x(10:12); % bias removed accel
    omega_ = omega - x(13:15); % bias removed gyro
    
    % --- Time update
    delOmega = Skew_symmetric(omega); 
    Cbn = Cbn*((2*eye(3)+delOmega*dt)/(2*eye(3)-delOmega*dt)); % update Cbn from (k) to (k+1)
    
    acc_n = Cbn*acc + [0;0;9.81]; % remove gravity to get linear acceleration in the navigation frame
    vel = vel + acc_n*dt; % update velocity from (k) to (k+1)
    pos = pos + vel*dt; % update position from (k) to (k+1)
    
    % Form state transition matrix and discrete process covariance matrix
    [PHI,Qd] = stm_and_qd( Cbn , acc_ , dt , Q ); 
    
    x = PHI*x; % propagate error state estimate
    P = PHI*P*PHI' + Qd; % propagate error state covariance matrix
    
    
    % --- Find best time of line
    while gps.time(gpsk)<=imu.time(k)
        
        H = measurement_jacobian(x,Cbn,u(:,k),prms);
        h = measurement_equation(x,pos,vel,Cbn,u(:,k),prms);

        H1 = H(1:3,:); H2 = H(4:6,:);
        h1 = h(1:3,:); h2 = h(4:6,:);
        
        H=[];R=[];m=[];h=[];
        if prms.POSITION_UPDATE
            H = [H;H1];
            R = blkdiag(R,R1);
            m = [m;pos - gps.pos(:,gpsk)];
            h = [h;h1];
        end
        if prms.VELOCITY_UPDATE
            H = [H;H2];
            R = blkdiag(R,R2);
            m = [m;vel - gps.vel(:,gpsk)];
            h = [h;h2];
        end
        if ~isempty(H)
            y = m-h;
            K = P*H'/(H*P*H' + R); % Kalman gain
            x = x + K*y; % update error state
            P = (eye(15) - K*H)*P*(eye(15) - K*H)'+K*R*K'; % update state estimate covariance matrix
        end
        
        h = measurement_equation(x,pos,vel,Cbn,u(:,k),prms);
        h = h(1:length(m));
        
        update_bool(k) = 1;
        
        resid_save(:,gpsk) = m-h;
        
        gpsk = gpsk + 1;
    end
    % --- Save best estimate (reference with error state corrections)
    pos_ref = pos - x(1:3);
    vel_ref = vel - x(4:6);
    Theta = Skew_symmetric(-x(7:9));
    Cbn_ref = ((2*I+Theta)/(2*I-Theta))*Cbn;
    bias = x(10:15);
    att_ref = CTM_to_Euler(Cbn_ref');
    
    est_save(:,k) = [pos_ref;vel_ref;att_ref;bias];
    var_save(:,k) = diag(P);
    err_save(:,k) = x(1:9);
    
    if prms.feedbackBool(k)
        % --- Feedback errors and zero out error state
        pos = pos_ref;
        vel = vel_ref;
        Cbn = Cbn_ref;

        x(1:9) = 0;
    end
    
end
est_save(:,1) = est_save(:,2);
var_save(:,1) = var_save(:,2);
err_save(:,1) = err_save(:,2);

%% -------- Package for delivery
est.time = time;

% --- Nav solution
est.pos = est_save(1:3,:);
est.vel = est_save(4:6,:);
est.att = est_save(7:9,:);
est.bias = est_save(10:15,:);

est.err = err_save;
est.update = logical(update_bool);


est.resid = resid_save;
est.variance = var_save;

end

function h = measurement_equation(x,pos,vel,Cbn,u,prm)

pos_ref = pos - x(1:3);
vel_ref = vel - x(4:6);
Theta = Skew_symmetric(-x(7:9));
Cbn_ref = ((2*eye(3)+Theta)/(2*eye(3)-Theta))*Cbn;
bias = x(10:15);

% --- Position measurement
r_ba_b = [prm.ox;prm.oy;prm.oz]; % Antenna offset in the vehicle body frame

r_b_n = pos_ref; % position of the vehicle in the nav frame

r_ba_n = Cbn_ref*r_ba_b; % Antenna offset in the nav frame

r_a_n = r_b_n + r_ba_n; % Antenna position in the nav frame

h_pos = pos - r_a_n;

% --- Velocity measurement
Omega = u(4:6) - bias(4:6);

v_b_n = vel_ref; % velocity of the CG resolved in the nav frame 

v_a_n = v_b_n + cross(Omega,r_ba_n);

h_vel = vel - v_a_n;

h = [h_pos;h_vel];

end

function H = measurement_jacobian(x,Cbn,u,prm)

att = CTM_to_Euler(Cbn');

% Bias estimates
bgx = x(13);
bgy = x(14);
bgz = x(15);

% Antenna offsets
ox = prm.ox;
oy = prm.oy;
oz = prm.oz;

% Gyroscope measurement
gyrx = u(4);
gyry = u(5);
gyrz = u(6);

omega = [gyrx;gyry;gyrz] - [bgx;bgy;bgz];
r_ba_b = [ox;oy;oz];

s_w=sin(att);
c_w=cos(att);

% --- rotation matrices
R_roll = [1,       0,      0; 
     0,  c_w(1), -s_w(1);
     0, s_w(1), c_w(1)];

R_pitch = [c_w(2), 0, s_w(2);
                0, 1,       0;
           -s_w(2), 0,  c_w(2)];

R_yaw = [ c_w(3), -s_w(3), 0;
    s_w(3), c_w(3), 0;
          0,      0, 1];

Cbn = R_yaw*R_pitch*R_roll;   % from nav to body

% --- partial derivatives of rotation matrices
dR_roll = [0,0,0;
           0,-s_w(1),-c_w(1);
           0,c_w(1),-s_w(1)];

dR_pitch = [-s_w(2),0,c_w(2);
              0,0,         0;
            -c_w(2),0,-s_w(2)];
        
dR_yaw = [-s_w(3),-c_w(3),0;
           c_w(3),-s_w(3),0;
           0,0,0];
       
dCbn_roll = R_yaw*R_pitch*dR_roll; % partial wrt roll
dCbn_pitch = R_yaw*dR_pitch*R_roll; % partial wrt pitch
dCbn_yaw = dR_yaw*R_pitch*R_roll; % partial wrt yaw

% --- form measurement Jacobian
H13 = [dCbn_roll*r_ba_b,dCbn_pitch*r_ba_b,dCbn_yaw*r_ba_b];

H23 = [cross(omega,dCbn_roll*r_ba_b),cross(omega,dCbn_pitch*r_ba_b),cross(omega,dCbn_yaw*r_ba_b)];

H25 = Skew_symmetric(Cbn*r_ba_b);

I = eye(3); O = zeros(3);

H = [I,O,H13,O,O;
     O,I,H23,O,H25];
    

end

function eul = CTM_to_Euler(C)
%CTM_to_Euler - Converts a coordinate transformation matrix to the
%corresponding set of Euler angles

eul(1,1) = atan2(C(2,3),C(3,3));  % roll
eul(2,1) = - asin(C(1,3));        % pitch
eul(3,1) = atan2(C(1,2),C(1,1));  % yaw

% Note: C is the transform from nav frame to the body
end

function [ R ] = RotationMatrix(euler)

    s_w=sin(euler);
    c_w=cos(euler);

   R_roll = [1,       0,      0; 
             0,  c_w(1), -s_w(1);
             0, s_w(1), c_w(1)];

   R_pitch = [c_w(2), 0, s_w(2);
                   0, 1,       0;
              -s_w(2), 0,  c_w(2)];
          
   R_yaw = [ c_w(3), -s_w(3), 0;
            s_w(3), c_w(3), 0;
                  0,      0, 1];
   
   R = R_yaw*R_pitch*R_roll;   % from nav to body
   
   % Note: R is the transform from body frame to the nav
end

function A = Skew_symmetric(a)
%Skew_symmetric - Calculates skew-symmetric matrix

A = [    0, -a(3),  a(2);...
      a(3),     0, -a(1);...
     -a(2),  a(1),     0];

end

function [ PHI , Qd] = stm_and_qd( Cbn , acc_ , dt , Q )

I = eye(3); O = zeros(3);

acc_n_ = Cbn*acc_;

% Calc S
S = Skew_symmetric(acc_n_); % skew symettric matrix

% ---- 
phi35 = dt*Cbn;

% ---- 
phi12 = dt*I;

% ---- 
phi23 = -dt*S;

% ---- 
phi24 = dt*Cbn;

% --- State Transition Matrix
PHI = [    I,phi12,    O,    O,    O;
           O,    I,phi23,phi24,    O;
           O,    O,    I,    O,phi35;
           O,    O,    O,    I,    O;
           O,    O,    O,    O,    I];

% --- Process Noise Matrix (continuous)
Bw = [  O,  O,  O,  O;
      Cbn,  O,  O,  O;
        O,Cbn,  O,  O;
        O,  O,  I,  O;
        O,  O,  O,  I];

% --- Process Noise Covariance Matrix (discrete)
Qd = Bw*Q*Bw'*dt;
         
end