#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state/prior state
   */
   x_ = F_*x_;
   P_ = F_*P_*F_.transpose();
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   // Calculate measurement residual or error
   VectorXd y = z - (H_*x_);
   // Calculate Kalman filter denominator
   MatrixXd s = H_*P*H_.transpose();
   // Calculate Kalman gain
   MatrixXd K = P_*H_.transpose()*s.inverse();
  
  // Calculate posterior or new state
  I = MatrixXd::Identity(2, 2);
  x_= x_ + K*y;
  P_ = (I-K*H)*P;
   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
