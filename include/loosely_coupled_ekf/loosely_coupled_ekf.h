/*
  \file		loosely_coupled_ekf.h
  \brief	Header file for Loosely Coupled EKF Algorithm.
  
  \author	Stephen Geiger <sag0020@tigermail.auburn.edu>
  \date		April 4, 2017
*/


#ifndef LOOSELY_COUPLED_EKF_H
#define LOOSELY_COUPLED_EKF_H

// ROS Includes
#include <ros/ros.h>
#include <ros/time.h>

// C++ Includes
#include <math.h>
#include <cmath>
#include <vector>

// Eigen Package include
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

// Message Include
#include <loosely_coupled_ekf/LooselyCoupledEstimate.h>

// Define PI and gpsPI
#ifndef PI
#define PI 3.141592653589793
#endif
#ifndef gpsPI
#define gpsPI 3.1415926535898
#endif

struct phi_qd
{	
	Eigen::MatrixXd PHI;
	Eigen::MatrixXd Qd;
};

class LooselyCoupledEKF
{
public:
	LooselyCoupledEKF();
	~LooselyCoupledEKF(){};

	void estimation(bool MEAS_UPDATE_COND);
    // Timer Callback
    void reset_error_state(const ros::TimerEvent& event);

	ros::Publisher estimatePub;

	// Initial State Vector
	Eigen::VectorXd X;

	// EKF Parameters Structure
	struct parameters
	{
		double std_pos;
		double std_vel;
		double std_att;
		double std_acc;
		double std_acc_bias;
		double std_gyr;
		double std_gyr_bias;
		double std_pos_i;
		double std_vel_i;
		double std_att_i;
		double std_acc_bias_i;
		double std_gyr_bias_i;
		std::vector <double> pos_i;
		std::vector <double> vel_i;
		std::vector <double> att_i;
		std::vector <double> bias_i;
	};
	double dt_integration;
	// Instance of parameters structure
	parameters prms;
	// Instance of estimate message
    loosely_coupled_ekf::LooselyCoupledEstimate estimate;

	// State Estimate Covariance Matrix
	Eigen::MatrixXd P;
	// Raw Data Vector
	Eigen::VectorXd Y;
	// Input
	Eigen::VectorXd u;
	// Measurement Model
	Eigen::MatrixXd H;
	// Weight Matrix
	Eigen::MatrixXd R;
	// Transformation Matrix
	Eigen::Matrix3d Cbn;
	// Identity Matrices
	Eigen::Matrix3d eye3; // 3x3 Identity
	Eigen::MatrixXd eye15; // 15x15 Identity
	// Zero Matrix
	Eigen::Matrix3d zero3;
	// Navigation frame position and velocity
	Eigen::Vector3d pos;
	Eigen::Vector3d vel;
	
private:
    void time_update(Eigen::VectorXd &X, Eigen::MatrixXd &P, Eigen::VectorXd u, Eigen::Matrix3d &Cbn, Eigen::MatrixXd Q, double dt);
	phi_qd stm_and_qd(Eigen::Matrix3d Cbn, Eigen::VectorXd u, Eigen::MatrixXd Q, double dt);
	void measurement_update(Eigen::VectorXd &X, Eigen::MatrixXd &P, Eigen::VectorXd m, Eigen::MatrixXd R, Eigen::MatrixXd H);
	void wrapToPi(double &angle);
	Eigen::Matrix3d skew_symmetric(Eigen::Vector3d accels);
	Eigen::Vector3d ctm_to_euler(Eigen::Matrix3d C);

//-----------------Estimation Parameters----------------------
	// Model Matrices
	Eigen::MatrixXd Bw;
	// 1-sigma values
	double sigma_wg;
	double sigma_course;
	double sigma_wb;
	// Weight Matrix
	Eigen::MatrixXd Q;
	// States
	Eigen::Vector3d att;
	Eigen::Vector3d b_a;
	Eigen::Vector3d b_g;
	// Measurement Vector
	Eigen::VectorXd m;
	// Inputs
	Eigen::Vector3d accels;
	Eigen::Vector3d gyros;
	// Gravity Vector
	Eigen::Vector3d gravity;
	// Unbiased accels
	Eigen::Vector3d accel_b;
	Eigen::Vector3d accel_n; // without gravity
	Eigen::Vector3d accel_n_; // with gravity
	// Unbiased gyros
	Eigen::Vector3d gyro_b;
	Eigen::Vector3d gyro_n;
	// Body Frame to Nav Frame Transformation
	Eigen::Matrix3d delOmega;
	// State Matrix and Qd
	phi_qd PHI_Qd;
	Eigen::Matrix3d S;
	Eigen::MatrixXd PHI;
	Eigen::Matrix3d phi12;
	Eigen::Matrix3d phi23;
	Eigen::Matrix3d phi24;
	Eigen::Matrix3d phi35;
	Eigen::MatrixXd Qd;
	// Kalman Gain
	Eigen::MatrixXd interm_to_K;
	Eigen::MatrixXd K_transpose;
	Eigen::MatrixXd K;

//-----------------Output Variables-----------------
	Eigen::Vector3d pos_ref;
	Eigen::Vector3d vel_ref;
	Eigen::Vector3d interm_to_Theta;
	Eigen::Matrix3d Theta;
	Eigen::Matrix3d Cbn_ref;
	Eigen::VectorXd bias;
	Eigen::Vector3d att_ref;

};


#endif