#include "kalman_filter.h"
#include "cmath"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in, MatrixXd &Hj_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  Hj_ = Hj_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state/prior state
   */
   x_ = F_*x_;
   P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   // Calculate measurement residual or error
   VectorXd y = z - (H_*x_);
   std::cout << "Measurement residual"<< y << std::endl;
   // Calculate Kalman gain denominator
   MatrixXd s = H_*P_*H_.transpose() + R_;
   std::cout << "Kalman gain denominator"<< s << std::endl;
   // Calculate Kalman gain
   MatrixXd K = P_*H_.transpose()*s.inverse();
   std::cout << "Kalman gain"<< K << std::endl;
  // Calculate posterior or new state
   MatrixXd I = MatrixXd::Identity(4, 4);
   std::cout << "Identity Matrix"<< I << std::endl;
   std::cout << "x_"<< x_ << std::endl;
   std::cout << "(K*y)"<< (K*y) << std::endl;
   x_= x_ + (K*y);
   P_ = (I-K*H_)*P_;
   
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */  
   VectorXd polar_state = tools.FromCartesianToPolar(x_);
   
  // Calculate measurement residual or error
   VectorXd y = z - polar_state;
   // Normalize angle
   while(y(1) > M_PI){
    y(1) -=2*M_PI; 
   }
   while(y(1) < -M_PI){
    y(1) +=2*M_PI; 
   }
   std::cout << "Measurement residual"<< y << std::endl;
   // Calculate Kalman filter denominator
   MatrixXd s = H_*P_*H_.transpose() + R_;
   std::cout << "Kalman gain denominator"<< s << std::endl;
   // Calculate Kalman gain
   MatrixXd K = P_*H_.transpose()*s.inverse();
   std::cout << "Kalman gain"<< K << std::endl;
  // Calculate posterior or new state
  MatrixXd I = MatrixXd::Identity(4, 4);
  std::cout << "Identity Matrix"<< I << std::endl;
  std::cout << "x_"<< x_ << std::endl;
  x_= x_ + K*y;
  std::cout << "x_"<< x_ << std::endl;
  P_ = (I-K*H_)*P_;

}
