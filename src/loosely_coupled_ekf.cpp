/*
  \file     loosely_coupled_ekf.cpp
  \brief    C++ file for Loosely Coupled EKF Algorithm.
  
  \authors  matlab development: Dan Pierce <jdp0009@tigermail.auburn.edu>
            ROS implementation: Stephen Geiger <sag0020@tigermail.auburn.edu>
  \date     April 4, 2017
*/


#include "loosely_coupled_ekf.h"

LooselyCoupledEKF::LooselyCoupledEKF(): PHI(15,15), Bw(15,12), Qd(15,15), Q(12,12), R(6,6), H(6,15), P(15,15), X(15), eye15(15,15), Y(6), m(6), u(6), bias(6)
{
	// Identity Matrix
	eye3.setIdentity();
	eye15.setIdentity();
    // Set Zero Matrix
    zero3.setZero();
	// Gravity Vector
	gravity << 0, 0, 9.81;
}

phi_qd LooselyCoupledEKF::stm_and_qd(Eigen::Matrix3d Cbn, Eigen::VectorXd u, Eigen::MatrixXd Q, double dt) {
    phi_qd out;

    // Unpack Inputs
    accels << u[0], u[1], u[2];
    accel_n_ = Cbn*accels;

    // Skew Symmetric matrix
    S = skew_symmetric(accel_n_);

    // Model Matrices
    phi12 = dt*eye3;
    phi23 = -dt*S;
    phi24 = dt*Cbn;
    phi35 = dt*Cbn;
    PHI << eye3,  phi12, zero3, zero3, zero3,
    	   zero3,  eye3, phi23, phi24, zero3,
    	   zero3, zero3,  eye3, zero3, phi35,
    	   zero3, zero3, zero3,  eye3, zero3,
    	   zero3, zero3, zero3, zero3,  eye3;
    Bw << zero3, zero3, zero3, zero3,
    	    Cbn, zero3, zero3, zero3,
    	  zero3,   Cbn, zero3, zero3,
    	  zero3, zero3,  eye3, zero3,
    	  zero3, zero3, zero3,  eye3;
    Qd = Bw * Q * Bw.transpose() * dt;

    out.PHI = PHI;
    out.Qd = Qd;

    return out;
}

void LooselyCoupledEKF::time_update(Eigen::VectorXd &X, Eigen::MatrixXd &P, Eigen::VectorXd u, Eigen::Matrix3d &Cbn, Eigen::MatrixXd Q, double dt) {
	// Unpack State
	b_a << X[9], X[10], X[11];
	b_g << X[12], X[13], X[14];

	// Unpack Input
	accels << u[0], u[1], u[2];
	gyros << u[3], u[4], u[5];

	// Remove Bias from Inputs
	accel_b = accels - b_a;
	gyro_b = gyros - b_g;

	// Gyro Transformation
	delOmega = skew_symmetric(gyros);
	Cbn = Cbn*(2*eye3 + delOmega*dt)*(2*eye3 - delOmega*dt).inverse();
    // Cbn = Cbn*tmp1*tmp2;

	// Remove Gravity to get Linear Accel in the Nav Frame
    if (estimate.header.frame_id == "NED") {
	    accel_n = Cbn*accels + gravity;
    } else if (estimate.header.frame_id == "ENU") {
        accel_n = Cbn*accels - gravity;
    }  

	// Update Velocity
	vel = vel + accel_n*dt;

	// Update Position
	pos = pos + vel*dt;

	// Form State Transition Matrix and Discrete Process Covariance Matrix
	PHI_Qd = stm_and_qd(Cbn, accel_b, Q, dt);
	// Propogate Error State Estimate
	X = PHI_Qd.PHI * X;

	// Propogate Error State Covariance Matrix
	P = PHI_Qd.PHI * P * PHI_Qd.PHI.transpose() + PHI_Qd.Qd;
}

void LooselyCoupledEKF::measurement_update(Eigen::VectorXd &X, Eigen::MatrixXd &P, Eigen::VectorXd m, Eigen::MatrixXd R, Eigen::MatrixXd H) {
    // Kalman Gain
    interm_to_K = H * P * H.transpose() + R;
	K_transpose = (interm_to_K.transpose()).fullPivLu().solve((P * H.transpose()).transpose());
    K = K_transpose.transpose();
	// Update Error State
	X = X + K * (m - H * X);

	// Update State Estimate Covariance Matrix
    P = (eye15 - K * H) * P * (eye15 - K * H).transpose() + K * R * K.transpose();
}

Eigen::Matrix3d LooselyCoupledEKF::skew_symmetric(Eigen::Vector3d accels) {
	// Software for use with "Principles of GNSS, Inertial, 
	// and Multisensor Integration Systems," Second Edition.
	//
	// This function created 1/4/2012 by Paul Groves
	// Copyright 2012, Paul Groves
	// License: BSD
    Eigen::Matrix3d A;

	A << 	  	  0, -accels[2],  accels[1],
		  accels[2], 		  0, -accels[0],
		 -accels[1],  accels[0], 		  0;
	return A;
}

Eigen::Vector3d LooselyCoupledEKF::ctm_to_euler(Eigen::Matrix3d C) {
    // Software for use with "Principles of GNSS, Inertial, 
    // and Multisensor Integration Systems," Second Edition.
    //
    // This function created 1/4/2012 by Paul Groves
    // Copyright 2012, Paul Groves
    // License: BSD
    Eigen::Vector3d eul;

    eul(0) = atan2(C(1,2),C(2,2));
    eul(1) = -asin(C(0,2));
    eul(2) = atan2(C(0,1),C(0,0));

    return eul;
}

void LooselyCoupledEKF::reset_error_state(const ros::TimerEvent& event)
{
    pos = pos_ref;
    vel = vel_ref;
    Cbn = Cbn_ref;

    for (int i = 0; i<9;i++)
    {
        X(i) = 0;
    }
}

void LooselyCoupledEKF::estimation(bool MEAS_UPDATE_COND) {
    //------------Noise Properties------------
    // Process Noise Covariance Matrix
    Q << pow(prms.std_acc,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, pow(prms.std_acc,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, pow(prms.std_acc,2), 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, pow(prms.std_gyr,2), 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, pow(prms.std_gyr,2), 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, pow(prms.std_gyr,2), 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, pow(prms.std_acc_bias,2), 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, pow(prms.std_acc_bias,2), 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, pow(prms.std_acc_bias,2), 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, pow(prms.std_gyr_bias,2), 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pow(prms.std_gyr_bias,2), 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pow(prms.std_gyr_bias,2);
    // Measurement Noise Covariance Matrix
    R << pow(prms.std_pos,2), 0, 0, 0, 0, 0,
         0, pow(prms.std_pos,2), 0, 0, 0, 0,
         0, 0, pow(prms.std_pos,2), 0, 0, 0,
         0, 0, 0, pow(prms.std_vel,2), 0, 0,
         0, 0, 0, 0, pow(prms.std_vel,2), 0,
         0, 0, 0, 0, 0, pow(prms.std_vel,2);

    // Perform Time Update if condition is false
    if (!MEAS_UPDATE_COND)
    {
        time_update(X,P,u,Cbn,Q,dt_integration);
    }

    // Perform Measurement Update if condition is true
    if (MEAS_UPDATE_COND)
    {
        m(0) = pos(0) - Y(0);
        m(1) = pos(1) - Y(1);
        m(2) = pos(2) - Y(2);
        m(3) = vel(0) - Y(3);
        m(4) = vel(1) - Y(4);
        m(5) = vel(2) - Y(5);
        measurement_update(X,P,m,R,H);
    }

    // Update Navigation Frame States
    pos_ref(0) = pos(0) - X(0);
    pos_ref(1) = pos(1) - X(1);
    pos_ref(2) = pos(2) - X(2);
    vel_ref(0) = vel(0) - X(3);
    vel_ref(1) = vel(1) - X(4);
    vel_ref(2) = vel(2) - X(5);
    interm_to_Theta << -X(6), -X(7), -X(8);
    Theta = skew_symmetric(interm_to_Theta);
    Cbn_ref = (2 * eye3 + Theta) * (2 * eye3 - Theta).inverse() * Cbn;
    bias << X(9), X(10), X(11), X(12), X(13), X(14);
    // Compute Attitude Euler angles
    att_ref = ctm_to_euler(Cbn_ref.transpose());

    // Publish to /EKF/estimate topic
    estimate.header.stamp = ros::Time::now();

    estimate.state.x = pos_ref(0);
    estimate.state.y = pos_ref(1);
    estimate.state.z = pos_ref(2);
    estimate.state.vx = vel_ref(0);
    estimate.state.vy = vel_ref(1);
    estimate.state.vz = vel_ref(2);
    estimate.state.roll = att_ref(0);
    estimate.state.pitch = att_ref(1);
    estimate.state.yaw = att_ref(2);
    estimate.state.bias_ax = bias(0);
    estimate.state.bias_ay = bias(1);
    estimate.state.bias_az = bias(2);
    estimate.state.bias_gx = bias(3);
    estimate.state.bias_gy = bias(4);
    estimate.state.bias_gz = bias(5);

    estimate.covariance.std_x = pow(P(0,0),0.5);
    estimate.covariance.std_y = pow(P(1,1),0.5);
    estimate.covariance.std_z = pow(P(2,2),0.5);
    estimate.covariance.std_vx = pow(P(3,3),0.5);
    estimate.covariance.std_vy = pow(P(4,4),0.5);
    estimate.covariance.std_vz = pow(P(5,5),0.5);
    estimate.covariance.std_roll = pow(P(6,6),0.5);
    estimate.covariance.std_pitch = pow(P(7,7),0.5);
    estimate.covariance.std_yaw = pow(P(8,8),0.5);
    estimate.covariance.std_bias_ax = pow(P(9,9),0.5);
    estimate.covariance.std_bias_ay = pow(P(10,10),0.5);
    estimate.covariance.std_bias_az = pow(P(11,11),0.5);
    estimate.covariance.std_bias_gx = pow(P(12,12),0.5);
    estimate.covariance.std_bias_gy = pow(P(13,13),0.5);
    estimate.covariance.std_bias_gz = pow(P(14,14),0.5);

    estimate.inputs.ax = u(0) - bias(0);
    estimate.inputs.ay = u(1) - bias(1);
    estimate.inputs.az = u(2) - bias(2);
    estimate.inputs.gx = u(3) - bias(3);
    estimate.inputs.gy = u(4) - bias(4);
    estimate.inputs.gz = u(5) - bias(5);

    estimatePub.publish(estimate);

}