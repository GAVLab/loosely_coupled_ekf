# loosely_coupled_ekf
6 DOF, loosely-coupled, error state EKF ROS node.


The 15 states are:

dX-, dY-, dZ-position (Navigation Frame)  [In m]
dX-, dY-, dZ-velocity (Navigation Frame)  [In m/s]
Roll, Pitch, Yaw Angle [In rad]
X-, Y-, Z-accel bias (Body Frame) [In m/s^2]
X-, Y-, Z-gyro bias  (Body Frame) [In rad/s]


The outputs of the ROS node are:

X-, Y-, Z-position (Navigation Frame)  [In m]
X-, Y-, Z-velocity (Navigation Frame)  [In m/s]
3 DOF Attitude
6 Biases
Variances on each state
6 Unbiased Inputs


In the launch file:

Set the Q & R weights
Set initial Covariance
Set initial position, velocity, attitude, and biases
Set GPS Sensor: ublox or novatel (Novatel yet to be tested)
Set IMU Sensor: xbow440
Set Navigation Frame: ENU or NED
Set receiving Position and/or Velocity Measurements