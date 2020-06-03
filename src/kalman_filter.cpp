#include "kalman_filter.h"
#include "cmath"

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
   MatrixXd s = H_*P_*H_.transpose();
   // Calculate Kalman gain
   MatrixXd K = P_*H_.transpose()*s.inverse();
  
  // Calculate posterior or new state
  MatrixXd I = MatrixXd::Identity(2, 2);
  x_= x_ + K*y;
  P_ = (I-K*H_)*P_;
   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    // create jacobian matrix for linearizing radar measurement
    MatrixXd Hj(4,4);
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];
    Hj << px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px,2)+pow(py,2)), 0, 0,
        -(py/(pow(px,2)+pow(py,2))),  px/(pow(px,2)+pow(py,2)),0,0,
        py*(vx*py-vy*px)/(pow(px,2)+pow(py,2),1.5), px*(vy*px-vx*py)/(pow(px,2)+pow(py,2),1.5), px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px,2)+pow(py,2));

  // Calculate measurement residual or error
   VectorXd y = z - (Hj*x_);
   // Calculate Kalman filter denominator
   MatrixXd s = H_*P_*H_.transpose();
   // Calculate Kalman gain
   MatrixXd K = P_*H_.transpose()*s.inverse();
  
  // Calculate posterior or new state
  MatrixXd I = MatrixXd::Identity(2, 2);
  x_= x_ + K*y;
  P_ = (I-K*H_)*P_;

}
