#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
   
   if(estimations.size()!=ground_truth.size() | estimations.size()==0){
        std::cout << "Invalid estimates or ground truth data" << std::endl;
        return rmse;
    }

   for(int i=0; i<estimations.size(); i++){
     VectorXd residual =  estimations[i] - ground_truth[i];
     
     residual = residual.array()*residual.array();
     rmse+= residual;
   }
   // compute mean
    rmse = rmse/estimations.size();
   // compute square root
   rmse = rmse.array().sqrt();
   
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(4,3);
     // Read  measurements
    float px = x_state[0];
    float py = x_state[1];
    float vx = x_state[2];
    float vy = x_state[3];

     // Check for division by zero
    if(fabs(px*px+py*py) < 0.0001){
	Hj = MatrixXd::Constant(4,4,0.0);
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
         return Hj;
	}
	
    // compute jacobian matrix for linearizing radar measurement
    Hj << px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px,2)+pow(py,2)), 0, 0,
        -(py/(pow(px,2)+pow(py,2))),  px/(pow(px,2)+pow(py,2)),0,0,
        py*(vx*py-vy*px)/(pow(px,2)+pow(py,2),1.5), px*(vy*px-vx*py)/(pow(px,2)+pow(py,2),1.5), px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px,2)+pow(py,2));

return Hj;
}
