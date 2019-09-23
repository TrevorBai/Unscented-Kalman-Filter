// Declare state updater class to filter data

#pragma once
#include <Eigen/Dense>
#include "UnscentedKF.h"
// We need retrieve measurement covariance for the updater
#include "UKFMeasurementPredictor.h"
// We need retrieve prior system state covariance for the updater
#include "UKFStatePredictor.h"

struct UnscentedKalmanFilterStateUpdater
{
	MatrixXd obtainKalmanFilterGain(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance);
	VectorXd postSystemStateVectorUpdator(VectorXd& odometerMeasurementVector, VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance);
	MatrixXd postSystemStateCovarianceUpdator(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance, VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance);
};