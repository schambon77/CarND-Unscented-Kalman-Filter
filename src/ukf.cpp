#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Parameters above this line are scaffolding, do not modify
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  Xsig_pred_ = MatrixXd(n_aug_, 2*n_aug_ + 1);
  time_us_ = 0;
  weights_ = VectorXd(2*n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  //-----------------
  //Initialization step
	  //-----------------
  if (!is_initialized_){
	// first measurement
	cout << "UKF: " << endl;
	x_ << 1, 1, 1, 1;

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	  /**
	  Convert radar from polar to cartesian coordinates and initialize state.
	  */
	  x_(0) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
	  x_(1) = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
	  //not enough info on velocity - leaving initial values
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
	  /**
	  Initialize state.
	  */
		x_(0) = meas_package.raw_measurements_(0);
		x_(1) = meas_package.raw_measurements_(1);
		//no info on velocity - leaving initial values
	}

	//Create covariance matrix
	P_ << 1, 0, 0, 0, 0,
			0, 1, 0, 0, 0,
			0, 0, 1000, 0, 0,
			0, 0, 0, 1000, 0,
			0, 0, 0, 0, 1000;

	time_us_ = meas_package.timestamp_;

	// done initializing, no need to predict or update
	is_initialized_ = true;
	return;
  }

  //-----------------
  //Prediction step
  //-----------------
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  cout << "dt = " << dt << endl;

  Prediction(dt);


  //-----------------
  //Measurement update step
  //-----------------

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	//-----------------
	//Generate sigma points
	//-----------------
	//create sigma point matrix
	MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

	//calculate square root of P
	MatrixXd A = P_.llt().matrixL();

	//set first column of sigma point matrix
	Xsig.col(0)  = x_;

	//set remaining sigma points
	for (int i = 0; i < n_x_; i++)
	{
	  Xsig.col(i+1)     = x_ + sqrt(lambda_ + n_x_) * A.col(i);
	  Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
	}

	//Predict sigma points

	//Predict state
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
