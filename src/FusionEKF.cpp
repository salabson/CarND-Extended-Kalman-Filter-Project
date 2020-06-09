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
  ekf_.P_ = MatrixXd(4,4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  H_laser_ << 1,0,0,0,
              0,1,0,0;
  

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
      
      ekf_.P_ << 1,0,0,0,
               0,1,0,0,
               0,0,1000,0,
               0,0,0,1000;
    

    ekf_.Q_ = MatrixXd(4,4);
    ekf_.F_ = MatrixXd(4,4);
    

    

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

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        VectorXd x_state = tools.FromPolarToCartesian(measurement_pack.raw_measurements_);
        double px = x_state(0);
    	double py = x_state(1);
    	double vx = x_state(2);
    	double vy = x_state(3);
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
  float c1 = pow(dt,2);
  float c2 = pow(dt,4)/4; 
  float c3 = pow(dt,3)/2; 
  ekf_.Q_ << c2*noise_ax, 0, c3*noise_ax, 0,
            0, c2*noise_ay, 0, c3*noise_ax,
            c3*noise_ax, 0, c1*noise_ax, 0,
            0, c3*noise_ay, 0, c1*noise_ay;

 
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
     //cout << "Radar measurementr" << measurement_pack.raw_measurements_ << endl;
        ekf_.H_ = MatrixXd(3, 4);
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
             
    // TODO: Radar updates
    // initialize jacobian matrix
   
    
    // Update radar measurement covariance
    ekf_.R_ = R_radar_;
    
    // update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // TODO: Laser updates
     
     ekf_.H_ = H_laser_;
     
    // Update laser measurement covariance
    ekf_.R_ = R_laser_;
     
    // update
    ekf_.Update(measurement_pack.raw_measurements_);
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
