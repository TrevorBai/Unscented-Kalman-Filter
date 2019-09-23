// Determine size of dynamic vector/matrix in this file

#include "UnscentedKF.h"
#include "UserInput.h"

UnscentedKalmanFilter::UnscentedKalmanFilter()
{
	//############################################################
	// State Space Parameters
	//############################################################
	priorSystemStateVector = VectorXd(STATE_SPACE_DIMENTION);
	postSystemStateVector = VectorXd(STATE_SPACE_DIMENTION);
	priorSystemStateCovariance = MatrixXd(STATE_SPACE_DIMENTION, STATE_SPACE_DIMENTION);
	postSystemStateCovariance = MatrixXd(STATE_SPACE_DIMENTION, STATE_SPACE_DIMENTION);
	processNoiseMatrix = MatrixXd(STATE_SPACE_DIMENTION, STATE_SPACE_DIMENTION);

	// unit is in meter
	processNoiseStandardDeviationInXDirection = 0.01;
	processNoiseStandardDeviationInYDirection = 0.02;
	// angle unit is in rad
	processNoiseStandardDeviationInThetaOrientation = 0.025;

	//############################################################
	// Measurement Space Parameters
	//############################################################
	odometerMeasurementVector = VectorXd(MEASUREMENT_SPACE_DIMENTION);
	measurementVector = VectorXd(MEASUREMENT_SPACE_DIMENTION);
	sensorNoiseMatrix = MatrixXd(MEASUREMENT_SPACE_DIMENTION, MEASUREMENT_SPACE_DIMENTION);
	measurementCovariance = MatrixXd(MEASUREMENT_SPACE_DIMENTION, MEASUREMENT_SPACE_DIMENTION);
	measurementAndPriorSystemStateCrossCovariance = MatrixXd(MEASUREMENT_SPACE_DIMENTION, MEASUREMENT_SPACE_DIMENTION);

	// unit is in meter
	measurementNoiseStandardDeviationInXDirection = 0.3;
	measurementNoiseStandardDeviationInYDirection = 0.16;
	// angle unit is in rad
	measurementNoiseStandardDeviationInThetaOrientation = 0.05;

	// Weighting Vector
	weightingVector = VectorXd(STATE_SPACE_DIMENTION);

	// Set up robot initial states. Attention, the initial robot
	// pose is considered as POST state vector after filtering
	postSystemStateVector << INITIAL_ROBOT_POSITION_X, INITIAL_ROBOT_POSITION_Y, INITIAL_ROBOT_ORIENTATION_THETA;
}

VectorXd UnscentedKalmanFilter::getPostSystemStateVector()
{
	return postSystemStateVector;
}

MatrixXd UnscentedKalmanFilter::getPostSystemStateCovariance()
{
	return postSystemStateCovariance;
}