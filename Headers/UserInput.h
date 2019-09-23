// This is an initialization header file, users need define
// dimension of state vector and measurement vector.
// Also initial expected robot pose and system control vector.

#pragma once
//############################################################
// Constants
//############################################################
constexpr auto PI = 3.141592653589793;

//############################################################
// User Input Section
//############################################################
constexpr auto STATE_SPACE_DIMENTION = 3;
constexpr auto INITIAL_ROBOT_POSITION_X = 1;
constexpr auto INITIAL_ROBOT_POSITION_Y = 2;
constexpr auto INITIAL_ROBOT_ORIENTATION_THETA = PI / 2;

// Measurement vector length should be equal to state space vector
// length. This is because we will need calculate cross covariance 
// in future. If they are not equal, we may have some issue regarding
// calculating matrix multiplication.
constexpr auto MEASUREMENT_SPACE_DIMENTION = 3;

//############################################################
// System Control Vector
//############################################################
constexpr auto LINEAR_VELOCITY = 0.3; // in m/s
constexpr auto ANGULAR_VELOCITY = 0.03;  // in rad/s
constexpr auto DELTA_T = 0.03; // in s