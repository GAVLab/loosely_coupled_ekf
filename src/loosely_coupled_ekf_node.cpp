/*
  \file     loosely_coupled_ekf_node.cpp
  \brief    C++ file for Loosely Coupled EKF ROS wrapper.
  
  \author   Stephen Geiger <sag0020@tigermail.auburn.edu>
  \date     April 4, 2017
*/


#include "loosely_coupled_ekf_node.h"

void HandleInfoMessages(const std::string &msg) 
{ 
    ROS_INFO("%s", msg.c_str()); 
};

void HandleWarningMessages(const std::string &msg) 
{
  ROS_WARN("%s", msg.c_str());
};

void HandleErrorMessages(const std::string &msg) 
{
  ROS_ERROR("%s", msg.c_str());
};

void HandleDebugMessages(const std::string &msg) 
{
  ROS_DEBUG("%s", msg.c_str());
};

LooselyCoupledNode::LooselyCoupledNode()
{

  	// Create Node Handles
  	ros::NodeHandle nh;
  	ros::NodeHandle nhPvt("~");

  	// Parameter Assignment from Launch file
    if( 
        // Measurement Noise Covariance Matrix (Q)
        !nhPvt.getParam("std_deviation_pos", EKF.prms.std_pos) || // Position Std Dev
        !nhPvt.getParam("std_deviation_vel", EKF.prms.std_vel) || // Velocity Std Dev
        // Process Noise Covariance Matrix (R)
        !nhPvt.getParam("std_deviation_acc", EKF.prms.std_acc) || // Accel Std Dev
        !nhPvt.getParam("std_deviation_acc_bias", EKF.prms.std_acc_bias) || // Accel Bias Std Dev
        !nhPvt.getParam("std_deviation_gyr", EKF.prms.std_gyr) || // Gyro Std Dev
        !nhPvt.getParam("std_deviation_gyr_bias", EKF.prms.std_gyr_bias) || // Gyro Bias Std Dev
        // Initial State Estimate Covariance Matrix (P)
        !nhPvt.getParam("std_deviation_pos_i", EKF.prms.std_pos_i) || // Pos Std Dev
        !nhPvt.getParam("std_deviation_vel_i", EKF.prms.std_vel_i) || // Vel Std Dev
        !nhPvt.getParam("std_deviation_att_i", EKF.prms.std_att_i) || // Attitude Std Dev
        !nhPvt.getParam("std_deviation_acc_bias_i", EKF.prms.std_acc_bias_i) || // Accel Bias Std Dev
        !nhPvt.getParam("std_deviation_gyr_bias_i", EKF.prms.std_gyr_bias_i) || // Gyro Bias Std Dev
        // Initial State Estimates (X)
        !nhPvt.getParam("initial_pos", EKF.prms.pos_i) || // Position
        !nhPvt.getParam("initial_vel", EKF.prms.vel_i) || // Velocity
        !nhPvt.getParam("initial_att", EKF.prms.att_i) || // Attitude
        !nhPvt.getParam("initial_bias", EKF.prms.bias_i) || // Bias
        // Integration Time step
        !nhPvt.getParam("dt_integration", EKF.dt_integration) ||
        // Sensors being used
        !nhPvt.getParam("GPS_SENSOR", GPS_SENSOR) ||
        !nhPvt.getParam("IMU_SENSOR", IMU_SENSOR) ||
        // Navigation Frame being used
        !nhPvt.getParam("NAV_FRAME", NAV_FRAME) ||
        // Receiving Position/Velocity Measurements
        !nhPvt.getParam("POS_MEASURE", POS_MEASURE) ||
        !nhPvt.getParam("VEL_MEASURE", VEL_MEASURE))
    {
        ROS_ERROR("Could not get all Error State EKF parameters");
    } 

    // Select Callback based on which sensors are being used
    if (GPS_SENSOR == 0)
    {
        // For Novatel Propak
        odomSub = nh.subscribe("novatel_node/odom", 10, &LooselyCoupledNode::novatel_odom_callback, this); // For novatel
    } else if (GPS_SENSOR == 1) {
        // For ublox
        odomSub = nh.subscribe("gps/navsol", 10, &LooselyCoupledNode::ublox_odom_callback, this);
        courseSub = nh.subscribe("gps/navvelned", 10, &LooselyCoupledNode::ublox_course_callback, this); // For ublox
    }
    if (IMU_SENSOR == 0) 
    {
        // For xbow 440
        imuSub = nh.subscribe("xbow440/imu/data", 10, &LooselyCoupledNode::xbow_callback, this); // For xbow 440
    }
    if (NAV_FRAME == 0)
    {
        EKF.estimate.header.frame_id = "ENU";
    } else if (NAV_FRAME == 1)
    {
        EKF.estimate.header.frame_id = "NED";
    }

    // Publish Estimates to "EKF/estimates" topic
    EKF.estimatePub = nh.advertise<loosely_coupled_ekf::LooselyCoupledEstimate>("EKF/estimate",10);

    // Initial State Estimate
    EKF.X << 0, 0, 0,
             0, 0, 0,
             0, 0, 0,
             EKF.prms.bias_i[0], EKF.prms.bias_i[1], EKF.prms.bias_i[2],
             EKF.prms.bias_i[3], EKF.prms.bias_i[4], EKF.prms.bias_i[5];

    // Initial Position and Velocity
    EKF.pos << EKF.prms.pos_i[0], EKF.prms.pos_i[1], EKF.prms.pos_i[2];
    EKF.vel << EKF.prms.vel_i[0], EKF.prms.vel_i[1], EKF.prms.vel_i[2];
    
    // Initial State Estimate Covariance Matrix
    EKF.P << pow(EKF.prms.std_pos_i,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, pow(EKF.prms.std_pos_i,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, pow(EKF.prms.std_pos_i,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, pow(EKF.prms.std_vel_i,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, pow(EKF.prms.std_vel_i,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, pow(EKF.prms.std_vel_i,2), 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, pow(EKF.prms.std_att_i,2), 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_att_i,2), 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_att_i,2), 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_acc_bias_i,2), 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_acc_bias_i,2), 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_acc_bias_i,2), 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_gyr_bias_i,2), 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_gyr_bias_i,2), 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, pow(EKF.prms.std_gyr_bias_i,2);

    // Initial Rotation Matrix
    EKF.Cbn = rotation_matrix(EKF.prms.att_i);

    // Initialize GPS Fix and Measurement Update Condition to False
    initLLA = false;
    MEAS_UPDATE_COND = false;            
}

void LooselyCoupledNode::ublox_course_callback(const ublox_msgs::NavVELNED& msg) 
{
    // For ublox Message
    ublox_course = msg.heading;

    if (initLLA) {

        // Vehicle Course [rad]
        course = (double)ublox_course * pow(10,-5) * PI / 180;
        ned2enuCourse(course); // Convert from NED frame to ENU frame
        wrapToPi(course); // Wrap course from -pi to +pi;

        // MEAS_UPDATE_COND = true;
        // if (MEAS_UPDATE_COND) {  
        //     EKF.estimation(MEAS_UPDATE_COND);
        //     MEAS_UPDATE_COND = false;
        // }
    }
    
}

void LooselyCoupledNode::ublox_odom_callback(const ublox_msgs::NavSOL& msg) 
{
    // For ublox Message
    if (initLLA) {
        ecef_pos[0] = (double)msg.ecefX / 100;
        ecef_pos[1] = (double)msg.ecefY / 100;
        ecef_pos[2] = (double)msg.ecefZ / 100;
        gnssCommon::wgsxyz2enu(enu_pos, ecef_pos, init_lla[0], init_lla[1], init_lla[2]);
        gnssCommon::wgsxyz2lla(lla[0], lla[1], lla[2], ecef_pos);
        ned_pos[0] = enu_pos[1];
        ned_pos[1] = enu_pos[0];
        ned_pos[2] = -enu_pos[2];

        ecef_vel[0] = (double)msg.ecefVX / 100;
        ecef_vel[1] = (double)msg.ecefVY / 100;
        ecef_vel[2] = (double)msg.ecefVZ / 100;
        ecef2enuVel(enu_vel, ecef_vel, lla[0], lla[1], lla[2]);
        ned_vel[0] = enu_vel[1];
        ned_vel[1] = enu_vel[0];
        ned_vel[2] = -enu_vel[2];

        if (NAV_FRAME == 0) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.Y[3] = enu_vel[0];
                EKF.Y[4] = enu_vel[1];
                EKF.Y[5] = enu_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_vel[0];
                EKF.Y[1] = enu_vel[1];
                EKF.Y[2] = enu_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        } else if (NAV_FRAME == 1) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.Y[3] = ned_vel[0];
                EKF.Y[4] = ned_vel[1];
                EKF.Y[5] = ned_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_vel[0];
                EKF.Y[1] = ned_vel[1];
                EKF.Y[2] = ned_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        }

        MEAS_UPDATE_COND = true;
        EKF.estimation(MEAS_UPDATE_COND);
        MEAS_UPDATE_COND = false;
    } else {
        ecef_pos[0] = (double)msg.ecefX / 100;
        ecef_pos[1] = (double)msg.ecefY / 100;
        ecef_pos[2] = (double)msg.ecefZ / 100;
        gnssCommon::wgsxyz2lla(init_lla[0], init_lla[1], init_lla[2], ecef_pos);
        gnssCommon::wgsxyz2enu(enu_pos, ecef_pos, init_lla[0], init_lla[1], init_lla[2]);
        ned_pos[0] = enu_pos[1];
        ned_pos[1] = enu_pos[0];
        ned_pos[2] = -enu_pos[2];

        ecef_vel[0] = (double)msg.ecefVX / 100;
        ecef_vel[1] = (double)msg.ecefVY / 100;
        ecef_vel[2] = (double)msg.ecefVZ / 100;
        ecef2enuVel(enu_vel, ecef_vel, init_lla[0], init_lla[1], init_lla[2]);
        ned_vel[0] = enu_vel[1];
        ned_vel[1] = enu_vel[0];
        ned_vel[2] = -enu_vel[2];

        if (NAV_FRAME == 0) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.Y[3] = enu_vel[0];
                EKF.Y[4] = enu_vel[1];
                EKF.Y[5] = enu_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_vel[0];
                EKF.Y[1] = enu_vel[1];
                EKF.Y[2] = enu_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        } else if (NAV_FRAME == 1) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.Y[3] = ned_vel[0];
                EKF.Y[4] = ned_vel[1];
                EKF.Y[5] = ned_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_vel[0];
                EKF.Y[1] = ned_vel[1];
                EKF.Y[2] = ned_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        }
        
        MEAS_UPDATE_COND = true;
        EKF.estimation(MEAS_UPDATE_COND);
        MEAS_UPDATE_COND = false;
        initLLA = true;
    }
}   

void LooselyCoupledNode::novatel_odom_callback(const nav_msgs::Odometry& msg) 
{
    // For Novatel Message
    if (initLLA) {
        ecef_pos[0] = (double)msg.pose.pose.position.x;
        ecef_pos[1] = (double)msg.pose.pose.position.y;
        ecef_pos[2] = (double)msg.pose.pose.position.z;
        gnssCommon::wgsxyz2enu(enu_pos, ecef_pos, init_lla[0], init_lla[1], init_lla[2]);
        gnssCommon::wgsxyz2lla(lla[0], lla[1], lla[2], ecef_pos);
        ned_pos[0] = enu_pos[1];
        ned_pos[1] = enu_pos[0];
        ned_pos[2] = -enu_pos[2];

        ecef_vel[0] = (double)msg.twist.twist.linear.x;
        ecef_vel[1] = (double)msg.twist.twist.linear.y;
        ecef_vel[2] = (double)msg.twist.twist.linear.z;
        ecef2enuVel(enu_vel, ecef_vel, lla[0], lla[1], lla[2]);
        ned_vel[0] = enu_vel[1];
        ned_vel[1] = enu_vel[0];
        ned_vel[2] = -enu_vel[2];

        if (NAV_FRAME == 0) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.Y[3] = enu_vel[0];
                EKF.Y[4] = enu_vel[1];
                EKF.Y[5] = enu_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_vel[0];
                EKF.Y[1] = enu_vel[1];
                EKF.Y[2] = enu_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        } else if (NAV_FRAME == 1) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.Y[3] = ned_vel[0];
                EKF.Y[4] = ned_vel[1];
                EKF.Y[5] = ned_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_vel[0];
                EKF.Y[1] = ned_vel[1];
                EKF.Y[2] = ned_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        }
        
        MEAS_UPDATE_COND = true;
        EKF.estimation(MEAS_UPDATE_COND);
        MEAS_UPDATE_COND = false;

    } else {
        ecef_pos[0] = (double)msg.pose.pose.position.x;
        ecef_pos[1] = (double)msg.pose.pose.position.y;
        ecef_pos[2] = (double)msg.pose.pose.position.z;
        gnssCommon::wgsxyz2lla(init_lla[0], init_lla[1], init_lla[2], ecef_pos);
        gnssCommon::wgsxyz2enu(enu_pos, ecef_pos, init_lla[0], init_lla[1], init_lla[2]);
        ned_pos[0] = enu_pos[1];
        ned_pos[1] = enu_pos[0];
        ned_pos[2] = -enu_pos[2];

        ecef_vel[0] = (double)msg.twist.twist.linear.x;
        ecef_vel[1] = (double)msg.twist.twist.linear.y;
        ecef_vel[2] = (double)msg.twist.twist.linear.z;
        ecef2enuVel(enu_vel, ecef_vel, init_lla[0], init_lla[1], init_lla[2]);
        ned_vel[0] = enu_vel[1];
        ned_vel[1] = enu_vel[0];
        ned_vel[2] = -enu_vel[2];

        if (NAV_FRAME == 0) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.Y[3] = enu_vel[0];
                EKF.Y[4] = enu_vel[1];
                EKF.Y[5] = enu_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = enu_pos[0];
                EKF.Y[1] = enu_pos[1];
                EKF.Y[2] = enu_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = enu_vel[0];
                EKF.Y[1] = enu_vel[1];
                EKF.Y[2] = enu_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        } else if (NAV_FRAME == 1) {
            if (POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.Y[3] = ned_vel[0];
                EKF.Y[4] = ned_vel[1];
                EKF.Y[5] = ned_vel[2];
                EKF.H <<  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3,
                         EKF.zero3,  EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (POS_MEASURE && !VEL_MEASURE) {
                EKF.Y[0] = ned_pos[0];
                EKF.Y[1] = ned_pos[1];
                EKF.Y[2] = ned_pos[2];
                EKF.H << EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3, EKF.zero3;
            } else if (!POS_MEASURE && VEL_MEASURE) {
                EKF.Y[0] = ned_vel[0];
                EKF.Y[1] = ned_vel[1];
                EKF.Y[2] = ned_vel[2];
                EKF.H << EKF.zero3, EKF.eye3, EKF.zero3, EKF.zero3, EKF.zero3;
            }
        }
        
        MEAS_UPDATE_COND = true;
        EKF.estimation(MEAS_UPDATE_COND);
        MEAS_UPDATE_COND = false;
        initLLA = true;
    }
}

void LooselyCoupledNode::xbow_callback(const sensor_msgs::Imu& msg)
{
    if (NAV_FRAME == 0) {
        roll_rate_raw = (double)msg.angular_velocity.x; // Roll Rate [rad/sec]
        pitch_rate_raw = -(double)msg.angular_velocity.y; // Pitch Rate [rad/sec]
        yaw_rate_raw = -(double)msg.angular_velocity.z;  // Yaw Rate  [rad/sec]
        accel_x_raw = (double)msg.linear_acceleration.x; // Longitudinal Acceleration [m/s^2]
        accel_y_raw = -(double)msg.linear_acceleration.y; // Lateral Acceleration [m/s^2]
        accel_z_raw = -(double)msg.linear_acceleration.z; // Vertical Acceleration [m/s^2]
    } else if (NAV_FRAME == 1) {
        roll_rate_raw = (double)msg.angular_velocity.x; // Roll Rate [rad/sec]
        pitch_rate_raw = (double)msg.angular_velocity.y; // Pitch Rate [rad/sec]
        yaw_rate_raw = (double)msg.angular_velocity.z;  // Yaw Rate  [rad/sec]
        accel_x_raw = (double)msg.linear_acceleration.x; // Longitudinal Acceleration [m/s^2]
        accel_y_raw = (double)msg.linear_acceleration.y; // Lateral Acceleration [m/s^2]
        accel_z_raw = (double)msg.linear_acceleration.z; // Vertical Acceleration [m/s^2]  
    }
    EKF.u << accel_x_raw, accel_y_raw, accel_z_raw, roll_rate_raw, pitch_rate_raw, yaw_rate_raw;
    EKF.estimation(MEAS_UPDATE_COND);
}

int LooselyCoupledNode::ecef2enuVel(double enu_vel[], double ecef_vel[], double lat, double lon, double alt)
{
    // // Have to change the type of the ecef velocity for the matrix math
    Eigen::Vector3d ecefVel(ecef_vel[0], ecef_vel[1], ecef_vel[2]);

    double R1_angle = 90 + lon;
    double R2_angle = 90 - lat;

    Eigen::Matrix3d R1 = rot(R1_angle, 3);
    Eigen::Matrix3d R2 = rot(R2_angle, 1);
    Eigen::Matrix3d Rot = R2*R1;

    Eigen::Vector3d enuVel = Rot * ecefVel;
    enu_vel[0] = enuVel(0);
    enu_vel[1] = enuVel(1);
    enu_vel[2] = enuVel(2);

    return (1);
}

Eigen::Matrix3d LooselyCoupledNode::rot(double angle, int axis)
{
    Eigen::Matrix3d R;

    double cang = cos(angle*gpsPI/180);
    double sang = sin(angle*gpsPI/180);

    if (axis==1) {
        R(0,0)=1;
        R(0,1)=0;
        R(0,2)=0;
        R(1,0)=0;
        R(1,1)=cang;
        R(1,2)=sang;
        R(2,0)=0;
        R(2,1)=-sang;
        R(2,2)=cang;
    }
    if (axis==2) {
        R(0,0)=cang;
        R(0,1)=0;
        R(0,2)=-sang;
        R(1,0)=0;
        R(1,1)=1;
        R(1,2)=0;
        R(2,0)=sang;
        R(2,1)=0;
        R(2,2)=cang;
    }
    if (axis==3) {
        R(0,0)=cang;
        R(0,1)=sang;
        R(0,2)=0;
        R(1,0)=-sang;
        R(1,1)=cang;
        R(1,2)=0;
        R(2,0)=0;
        R(2,1)=0;
        R(2,2)=1;
    }

    return R;
}

Eigen::Matrix3d LooselyCoupledNode::rotation_matrix(std::vector <double> euler) 
{
    Eigen::Matrix3d R;
    Eigen::Matrix3d R_roll;
    Eigen::Matrix3d R_pitch;
    Eigen::Matrix3d R_yaw;

    Eigen::Vector3d s_w(sin(euler[0]),sin(euler[1]),sin(euler[2]));
    Eigen::Vector3d c_w(cos(euler[0]),cos(euler[1]),cos(euler[2]));

    R_roll << 1,       0,      0,
              0,  c_w[0], s_w[0],
              0, -s_w[0], c_w[0];
    R_pitch << c_w[1], 0, -s_w[1],
                    0, 1,       0,
               s_w[1], 0,  c_w[1];
    R_yaw <<  c_w[2], s_w[2], 0,
             -s_w[2], c_w[2], 0,
                   0,      0, 1;

    R = R_roll * R_pitch * R_yaw;
    // R = R.transposeInPlace();
    return R.transpose();
}

void LooselyCoupledNode::ned2enuCourse(double &course)
{
    if (course >= 0 && course < gpsPI/2)
    {
        course = gpsPI/2 - course;
    } else {
        course = 5*gpsPI/2 - course;
    }
}

void LooselyCoupledNode::wrapToPi(double &angle) 
{
    while (angle > gpsPI) {
        angle = angle - 2 * gpsPI;
    }
    while (angle < -gpsPI) {
        angle = angle + 2 * gpsPI;
    }
}

void LooselyCoupledNode::wrapTo2Pi(double &angle) 
{
    while (angle > 2 * gpsPI) {
        angle = angle - 2 * gpsPI;
    }
    while (angle < 0) {
        angle = angle + 2 * gpsPI;
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS node
	ros::init(argc, argv, "EKF_node");
    
    // Create Instance of LooselyCoupledNode Class
    LooselyCoupledNode node;

    ros::spin();
	return 0;
}