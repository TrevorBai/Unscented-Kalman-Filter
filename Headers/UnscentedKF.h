//This header file declares all parameters regarding to UKF.
//Dynamic size matrix and vector are utilized, users are able
//to determine the size of them in UnscentedKF.cpp.

#pragma once
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct UnscentedKalmanFilter{

	// Declare Standard UKF Parameters

	//############################################################
	// State Space Parameters
	//############################################################
	VectorXd priorSystemStateVector;
	VectorXd postSystemStateVector;
	MatrixXd priorSystemStateCovariance;
	MatrixXd postSystemStateCovariance;
	MatrixXd processNoiseMatrix;
	double processNoiseStandardDeviationInXDirection;
	double processNoiseStandardDeviationInYDirection;
	double processNoiseStandardDeviationInThetaOrientation;
	
	//############################################################
	// Measurement Space Parameters
	//############################################################
	VectorXd odometerMeasurementVector;
	VectorXd measurementVector;
	MatrixXd sensorNoiseMatrix;
	MatrixXd measurementCovariance;
	MatrixXd measurementAndPriorSystemStateCrossCovariance;

	// Regarding measurement noise, types of sensors are not clarified.
	// User need properly evaluate the standard deviation of whatever sensors used
	double measurementNoiseStandardDeviationInXDirection;
	double measurementNoiseStandardDeviationInYDirection;
	double measurementNoiseStandardDeviationInThetaOrientation;

	//############################################################
	// Weighting Vector
	//############################################################
	VectorXd weightingVector;
	
	//############################################################
	// Default constructor will be initialized in UnscentedKF.cpp
	//############################################################
	UnscentedKalmanFilter();
	
	//############################################################
	// Declare Some Methods
	//############################################################
	VectorXd getPostSystemStateVector();
	MatrixXd getPostSystemStateCovariance();
};
