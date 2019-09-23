// Implement each method in state updater class

#include "UKFStateUpdater.h"
#include "UserInput.h"

// For filtering data, we need use normal Kalman filter to proceed it.
MatrixXd UnscentedKalmanFilterStateUpdater::obtainKalmanFilterGain(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance)
{
	// First, we need to obtain the normal Kalman fitler gain
	UnscentedKalmanFilterMeasurementPredictor unscentedKalmanFilterMeasurementPredictor;

	MatrixXd kalmanFilterGain = unscentedKalmanFilterMeasurementPredictor.measurementAndPriorSystemStateCrossCovariancePredictor(priorSystemStateVector, priorSystemStateCovariance) * unscentedKalmanFilterMeasurementPredictor.measurementCovariancePredictor(priorSystemStateVector, priorSystemStateCovariance).inverse();
	
	return kalmanFilterGain;
}

VectorXd UnscentedKalmanFilterStateUpdater::postSystemStateVectorUpdator(VectorXd& odometerMeasurementVector, VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance)
{
	// This method will update the post system state vector by filtering data
	UnscentedKalmanFilter unscentedKalmanFilter;
	UnscentedKalmanFilterMeasurementPredictor unscentedKalmanFilterMeasurementPredictor;

	unscentedKalmanFilter.postSystemStateVector = unscentedKalmanFilter.priorSystemStateVector + obtainKalmanFilterGain(priorSystemStateVector, priorSystemStateCovariance) * (unscentedKalmanFilter.odometerMeasurementVector - unscentedKalmanFilterMeasurementPredictor.measurementVectorPredictor(priorSystemStateVector, priorSystemStateCovariance));

	return unscentedKalmanFilter.postSystemStateVector;
}

MatrixXd UnscentedKalmanFilterStateUpdater::postSystemStateCovarianceUpdator(VectorXd& priorSystemStateVector, MatrixXd& priorSystemStateCovariance, VectorXd& postSystemStateVector, MatrixXd& postSystemStateCovariance)
{
	// This method will update post system state covariance by filtering data
	UnscentedKalmanFilter unscentedKalmanFilter;
	UnscentedKalmanFilterStatePredictor unscentedKalmanFilterStatePredictor;
	UnscentedKalmanFilterMeasurementPredictor unscentedKalmanFilterMeasurementPredictor;

	unscentedKalmanFilter.postSystemStateCovariance = unscentedKalmanFilterStatePredictor.priorSystemStateCovariancePredictor(postSystemStateVector, postSystemStateCovariance) - obtainKalmanFilterGain(priorSystemStateVector, priorSystemStateCovariance) * unscentedKalmanFilterMeasurementPredictor.measurementCovariancePredictor(priorSystemStateVector, priorSystemStateCovariance) * obtainKalmanFilterGain(priorSystemStateVector, priorSystemStateCovariance).transpose();

	return unscentedKalmanFilter.postSystemStateCovariance;
}