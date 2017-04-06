# EKF Estimate Covariance Message Structure

# Position Standard Deviation
float64 std_x
float64 std_y
float64 std_z

# Velocity Standard Deviation
float64 std_vx
float64 std_vy
float64 std_vz

# Attitude Standard Deviation
float64 std_roll
float64 std_pitch
float64 std_yaw

# Acceleration Bias Standard Deviation
float64 std_bias_ax
float64 std_bias_ay
float64 std_bias_az

# Gyro Bias Standard Deviation
float64 std_bias_gx
float64 std_bias_gy
float64 std_bias_gz