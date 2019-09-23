// Declare measurement predictor class

#pragma once
#include <Eigen/Dense>
#include "UnscentedKF.h"

struct UnscentedKalmanFilterMeasurementPredictor
{
	MatrixXd obtainInitialSigmaPointMatrix(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance);
	MatrixXd obtainUpdatedSigmaPointMatrix(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance);
	VectorXd calculateWeightingVector();
	VectorXd measurementVectorPredictor(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance);
	MatrixXd obtainMeasurementNoiseCovarianceMatrix();
	MatrixXd measurementCovariancePredictor(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance);
	MatrixXd measurementAndPriorSystemStateCrossCovariancePredictor(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance);
};

