// Declare state predictor class

#pragma once
#include <Eigen/Dense>
#include "UnscentedKF.h"

struct UnscentedKalmanFilterStatePredictor
{
	MatrixXd obtainInitialSigmaPointMatrix(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance);
	MatrixXd obtainUpdatedSigmaPointMatrix(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance);
	VectorXd calculateWeightingVector();
	VectorXd priorSystemStateVectorPredictor(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance);
	MatrixXd obtainProcessNoiseCovarianceMatrix();
	MatrixXd priorSystemStateCovariancePredictor(VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance);
};