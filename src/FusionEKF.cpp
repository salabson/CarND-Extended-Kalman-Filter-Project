#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
    ekf_.H_ = MatrixXd(2,4);
    ekf_.H_ << 1,0,0,0,
          0,1,0,0;

    ekf_.Q_ = MatrixXd(3,4);
    ekf_.F_ = MatrixXd(4,4);
    ekf_.P_ = MatrixXd(2,2);
    ekf_.R_ = MatrixXd(2,2);
    ekf_.Hj_= Hj_;
   

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1,1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        float px = measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
    	float py = measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);
    	float vx = measurement_pack.raw_measurements_[2]*cos(measurement_pack.raw_measurements_[1]);
    	float vy = measurement_pack.raw_measurements_[2]*sin(measurement_pack.raw_measurements_[1]);
	ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    }
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  // calcuate delta time
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // update state transition matrix to include elapsed tine 
  ekf_.F_ << 1,0,dt,0,
             0,1,0,dt,
             0,0,1,0,
             0,0,0,1;
  // update process noise covariance matrix
  ekf_.Q_ << (pow(dt,4)/4)*noise_ax, 0, (pow(dt,3)/2)*noise_ax, 0,
            0, (pow(dt,4)/4)*noise_ay, 0, (pow(dt,3)/2)*noise_ax,
            (pow(dt,3)/2)*noise_ax, 0, pow(dt,2)*noise_ax, 0,
            0, (pow(dt,3)/2)*noise_ay, 0, pow(dt,2)*noise_ay;

 
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    
    // Read radar measurements
    float px_r = measurement_pack.raw_measurements_[0];
    float py_r = measurement_pack.raw_measurements_[1];
    float vx_r = measurement_pack.raw_measurements_[2];
    float vy_r = measurement_pack.raw_measurements_[3];
  
    // Check for division by zero
    if(fabs(px_r*px_r+py_r*py_r) < 0.0001){
	ekf_.Hj_ = MatrixXd::Constant(4,4,0.0);
	}
	
    // create jacobian matrix for linearizing radar measurement
    ekf_.Hj_ << px_r/sqrt(pow(px_r,2)+pow(py_r,2)), py_r/sqrt(pow(px_r,2)+pow(py_r,2)), 0, 0,
        -(py_r/(pow(px_r,2)+pow(py_r,2))),  px_r/(pow(px_r,2)+pow(py_r,2)),0,0,
        py_r*(vx_r*py_r-vy_r*px_r)/(pow(px_r,2)+pow(py_r,2),1.5), px_r*(vy_r*px_r-vx_r*py_r)/(pow(px_r,2)+pow(py_r,2),1.5), px_r/sqrt(pow(px_r,2)+pow(py_r,2)), py_r/sqrt(pow(px_r,2)+pow(py_r,2));
    
    // Update radar measurement covariance
    ekf_.R_ << R_radar_;
    // update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    // Update laser measurement covariance
    ekf_.R_ << R_laser_;
    // update
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
