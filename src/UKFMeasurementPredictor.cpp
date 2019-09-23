// Implement measurement predictor class

#include "UKFMeasurementPredictor.h"
#include "UserInput.h"

MatrixXd UnscentedKalmanFilterMeasurementPredictor::obtainInitialSigmaPointMatrix(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance)
{
	MatrixXd sigmaPointMatrix(STATE_SPACE_DIMENTION, 2 * STATE_SPACE_DIMENTION);

	// #########################################################################
	// Sample sigma points again based on estimated system state and covariance
	// #########################################################################

	// Analytic formulas to sample sigma points are the same as before.
	// In order to save computational effort, we may omit this process.
	// Then use previous propagated sigma points.

	// Assuming we are sampling sigma points again, and they are shown below
	// It could be more scalable
	// Sample sigma point 1
	sigmaPointMatrix(0, 0) = 0.2;
	sigmaPointMatrix(1, 0) = 0.2;
	sigmaPointMatrix(2, 0) = 0.02;

	// Sample sigma point 2
	sigmaPointMatrix(0, 1) = 0.12;
	sigmaPointMatrix(1, 1) = 0.23;
	sigmaPointMatrix(2, 1) = 0.056;

	// Sample sigma point 3
	sigmaPointMatrix(0, 2) = 0.32;
	sigmaPointMatrix(1, 2) = 0.13;
	sigmaPointMatrix(2, 2) = 0.028;

	// Sample sigma point 4
	sigmaPointMatrix(0, 3) = 0.15;
	sigmaPointMatrix(1, 3) = 0.27;
	sigmaPointMatrix(2, 3) = 0.006;

	// Sample sigma point 5
	sigmaPointMatrix(0, 4) = 0.24;
	sigmaPointMatrix(1, 4) = 0.33;
	sigmaPointMatrix(2, 4) = 0.016;

	// Sample sigma point 6
	sigmaPointMatrix(0, 5) = 0.12;
	sigmaPointMatrix(1, 5) = 0.27;
	sigmaPointMatrix(2, 5) = 0.050;

	return sigmaPointMatrix;
}

MatrixXd UnscentedKalmanFilterMeasurementPredictor::obtainUpdatedSigmaPointMatrix(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance)
{
	static MatrixXd sigmaPointMatrix = obtainInitialSigmaPointMatrix(priorSystemStateVector, priorSystemStateCovariance);

	// Now map these sigma points to measurement space.
	// The mapping relation should be known beforehand.
	
	// TODO: Assuming the mapping relation is y = foo(x, delta_T).
	// User need to implement this method using sensor data
	// For example, Lidar and Radar sensor data.

	MatrixXd mappedSigmaPointMatrixInMeasurementSpace(MEASUREMENT_SPACE_DIMENTION, MEASUREMENT_SPACE_DIMENTION);

	return mappedSigmaPointMatrixInMeasurementSpace;
}

VectorXd UnscentedKalmanFilterMeasurementPredictor::calculateWeightingVector()
{
	// #######################################################################
	// For different approaches to computing a set of sigma points,
	// different weight calculations are derived.
	// User can use this method to implement different weighting calculations
	// ########################################################################
	
	// Standard UKF weighting calculations are implemented here
	double weight = static_cast <double>(1) / (2 * MEASUREMENT_SPACE_DIMENTION);
	VectorXd weightingVector(MEASUREMENT_SPACE_DIMENTION);
	weightingVector = weight * VectorXd::Ones(3);
	return weightingVector;
}

VectorXd UnscentedKalmanFilterMeasurementPredictor::measurementVectorPredictor(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance)
{
	UnscentedKalmanFilter unscentedKalmanFilter;
	MatrixXd mappedSigmaPointMatrixInMeasurementSpace = obtainUpdatedSigmaPointMatrix(priorSystemStateVector, priorSystemStateCovariance);

	// Calculate expected measurement vector.
	// Because each sigma points are equally weighted, we can use
	// Eigen built-in function mean() to retrieve the result 
	unscentedKalmanFilter.measurementVector = mappedSigmaPointMatrixInMeasurementSpace.rowwise().mean();
	return unscentedKalmanFilter.measurementVector;
}

MatrixXd UnscentedKalmanFilterMeasurementPredictor::obtainMeasurementNoiseCovarianceMatrix()
{
	UnscentedKalmanFilter unscentedKalmanFilter;
	// User need obtain the measurement noise covariance matrix
	// and type in here. Assuming it is
	unscentedKalmanFilter.sensorNoiseMatrix << 0.003, 0, 0,
											   0, 0.012, 0,
											   0, 0, 0.015;
	return unscentedKalmanFilter.sensorNoiseMatrix;
}

MatrixXd UnscentedKalmanFilterMeasurementPredictor::measurementCovariancePredictor(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance)
{
	// Calculate prior system state covariance matrix before filtering
	UnscentedKalmanFilter unscentedKalmanFilter;
	MatrixXd mappedSigmaPointMatrixInMeasurementSpace = obtainUpdatedSigmaPointMatrix(priorSystemStateVector, priorSystemStateCovariance);
	VectorXd measurementVector = measurementVectorPredictor(priorSystemStateVector, priorSystemStateCovariance);

	MatrixXd diveationMatrixForEachSigmaPoint = mappedSigmaPointMatrixInMeasurementSpace.colwise() - measurementVector;

	MatrixXd measurementCovariance = (diveationMatrixForEachSigmaPoint * diveationMatrixForEachSigmaPoint.transpose()) / double(2 * MEASUREMENT_SPACE_DIMENTION) + obtainMeasurementNoiseCovarianceMatrix();

	unscentedKalmanFilter.measurementCovariance = measurementCovariance;

	return unscentedKalmanFilter.measurementCovariance;
}

MatrixXd UnscentedKalmanFilterMeasurementPredictor::measurementAndPriorSystemStateCrossCovariancePredictor(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance)
{
	// Cross covariance matrix regarding measurement and prior system state
	// need be calculated because at next stage, which is UKF updater. We
	// will filter the process wrt the measurement to obtain optimal post
	// system state estimation.
	
	UnscentedKalmanFilter unscentedKalmanFilter;

	MatrixXd sigmaPointMatrix = obtainInitialSigmaPointMatrix(priorSystemStateVector, priorSystemStateCovariance);

	MatrixXd mappedSigmaPointMatrixInMeasurementSpace = obtainUpdatedSigmaPointMatrix(priorSystemStateVector, priorSystemStateCovariance);
	VectorXd measurementVector = measurementVectorPredictor(priorSystemStateVector, priorSystemStateCovariance);

	MatrixXd diveationMatrixOfOriginalSigmaPoint = sigmaPointMatrix.colwise() - unscentedKalmanFilter.priorSystemStateVector;

	MatrixXd diveationMatrixForEachSigmaPoint = mappedSigmaPointMatrixInMeasurementSpace.colwise() - measurementVector;

	MatrixXd measurementAndPriorSystemStateCrossCovariance = (diveationMatrixOfOriginalSigmaPoint * diveationMatrixForEachSigmaPoint.transpose()) / double(2 * MEASUREMENT_SPACE_DIMENTION);

	unscentedKalmanFilter.measurementAndPriorSystemStateCrossCovariance = measurementAndPriorSystemStateCrossCovariance;
	
	return unscentedKalmanFilter.measurementAndPriorSystemStateCrossCovariance;
}
