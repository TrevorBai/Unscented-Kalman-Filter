// Implement state predictor class

#include "UKFStatePredictor.h"
#include "UserInput.h"
#include "unsupported/Eigen/MatrixFunctions"

MatrixXd UnscentedKalmanFilterStatePredictor::obtainInitialSigmaPointMatrix(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance)
{
	MatrixXd sigmaPointMatrix(STATE_SPACE_DIMENTION, 2 * STATE_SPACE_DIMENTION);

	// #########################################################
	// Sample sigma points
	// #########################################################
	
	// Standard Unscented Kalman filter is implemented. 
	// First obtain 2 x STATE_SPACE_DIMENTION sample sigma points.
	// We may be gonna recursively use these points. It is better to
	// choose these points based on a set of analytic formulas.

	MatrixXd complexSqureRootMatrix = MatrixBase::sqrt(STATE_SPACE_DIMENTION * postSystemStateCovariance);

	MatrixXd transposeOfComplexSqureRootMatrix = complexSqureRootMatrix.transpose();
	
	for (int column = 0; column < STATE_SPACE_DIMENTION; column++)
	{
		sigmaPointMatrix.col(column) = postSystemStateVector + transposeOfComplexSqureRootMatrix.row(column);
	}

	for (int column = STATE_SPACE_DIMENTION; column < (2 * STATE_SPACE_DIMENTION); column++)
	{
		sigmaPointMatrix.col(column) = postSystemStateVector - transposeOfComplexSqureRootMatrix.row(column);
	}

	//// Assuming following sigma points are rational
	//// It could be more scalable
	//// Sample sigma point 1
	//sigmaPointMatrix(0, 0) = 0.2;
	//sigmaPointMatrix(1, 0) = 0.2;
	//sigmaPointMatrix(2, 0) = 0.02;

	//// Sample sigma point 2
	//sigmaPointMatrix(0, 1) = 0.12;
	//sigmaPointMatrix(1, 1) = 0.23;
	//sigmaPointMatrix(2, 1) = 0.056;

	//// Sample sigma point 3
	//sigmaPointMatrix(0, 2) = 0.32;
	//sigmaPointMatrix(1, 2) = 0.13;
	//sigmaPointMatrix(2, 2) = 0.028;

	//// Sample sigma point 4
	//sigmaPointMatrix(0, 3) = 0.15;
	//sigmaPointMatrix(1, 3) = 0.27;
	//sigmaPointMatrix(2, 3) = 0.006;

	//// Sample sigma point 5
	//sigmaPointMatrix(0, 4) = 0.24;
	//sigmaPointMatrix(1, 4) = 0.33;
	//sigmaPointMatrix(2, 4) = 0.016;

	//// Sample sigma point 6
	//sigmaPointMatrix(0, 5) = 0.12;
	//sigmaPointMatrix(1, 5) = 0.27;
	//sigmaPointMatrix(2, 5) = 0.050;

	return sigmaPointMatrix;
}

MatrixXd UnscentedKalmanFilterStatePredictor::obtainUpdatedSigmaPointMatrix(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance)
{
	static MatrixXd sigmaPointMatrix = obtainInitialSigmaPointMatrix(postSystemStateVector, postSystemStateCovariance);

	// Now propagate these sigma points based on system model.
	// We are using a simplified unicycle kinematic model
	// Attention, we do NOT need consider process noise here, because
	// it will be added in our covariance matrix.

	// Thus this propagation of sigma points are pure theoretical.
	for (int column = 0; column < sigmaPointMatrix.cols(); column++)
	{
		sigmaPointMatrix(0, column) = sigmaPointMatrix(0, column) + LINEAR_VELOCITY * DELTA_T * cos(sigmaPointMatrix(2, column));

		sigmaPointMatrix(1, column) = sigmaPointMatrix(1, column) + LINEAR_VELOCITY * DELTA_T * sin(sigmaPointMatrix(2, column));

		sigmaPointMatrix(2, column) = sigmaPointMatrix(2, column) + ANGULAR_VELOCITY * DELTA_T;
	}
	return sigmaPointMatrix;
}

VectorXd UnscentedKalmanFilterStatePredictor::calculateWeightingVector()
{
	// #######################################################################
	// For different approaches to computing a set of sigma points,
	// different weight calculations are derived.
	// User can use this method to implement different weighting calculations
	// ########################################################################

	// Standard UKF weighting calculations are implemented here
	double weight = static_cast <double>(1) / (2 * STATE_SPACE_DIMENTION);
	VectorXd weightingVector(STATE_SPACE_DIMENTION);
	weightingVector = weight * VectorXd::Ones(3);
	return weightingVector;
}

VectorXd UnscentedKalmanFilterStatePredictor::priorSystemStateVectorPredictor(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance)
{
	UnscentedKalmanFilter unscentedKalmanFilter;
	MatrixXd sigmaPointMatrix = obtainUpdatedSigmaPointMatrix(postSystemStateVector, postSystemStateCovariance);
	
	// Calculate prior system state vector
	// Because each sigma points are equally weighted, we can use
	// Eigen built-in function mean() to retrieve the result 
	unscentedKalmanFilter.priorSystemStateVector = sigmaPointMatrix.rowwise().mean();
	return unscentedKalmanFilter.priorSystemStateVector;
}

MatrixXd UnscentedKalmanFilterStatePredictor::obtainProcessNoiseCovarianceMatrix()
{
	UnscentedKalmanFilter unscentedKalmanFilter;
	// User need obtain the process noise covariance matrix
	// and type in here. Assuming it is
	unscentedKalmanFilter.processNoiseMatrix << 0.003, 0, 0,
												0, 0.012, 0,
												0, 0, 0.015;
	return unscentedKalmanFilter.processNoiseMatrix;
}

MatrixXd UnscentedKalmanFilterStatePredictor::priorSystemStateCovariancePredictor(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance)
{
	// Calculate prior system state covariance matrix before filtering
	UnscentedKalmanFilter unscentedKalmanFilter;
	MatrixXd sigmaPointMatrix = obtainUpdatedSigmaPointMatrix(postSystemStateVector, postSystemStateCovariance);
	VectorXd priorSystemStateVector = priorSystemStateVectorPredictor(postSystemStateVector, postSystemStateCovariance);
	
	MatrixXd diveationMatrixForEachSigmaPoint = sigmaPointMatrix.colwise() - priorSystemStateVector;

	MatrixXd priorSystemStateCovariance = (diveationMatrixForEachSigmaPoint * diveationMatrixForEachSigmaPoint.transpose()) / double(2 * STATE_SPACE_DIMENTION) + obtainProcessNoiseCovarianceMatrix();

	unscentedKalmanFilter.priorSystemStateCovariance = priorSystemStateCovariance;

	return unscentedKalmanFilter.priorSystemStateCovariance;
}