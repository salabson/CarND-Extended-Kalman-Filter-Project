#include "tools.h"
#include <iostream>
#include <math.h>

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
   
   if((estimations.size()!=ground_truth.size()) | (estimations.size()==0)){
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
    std::cout << " Start .. Calculate Jacobian"  << std::endl;
    
    MatrixXd Hj(3,4);
   // Read  measurements
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
   

     // Check for division by zero
    if(fabs(px*px+py*py) < 0.0001){
	Hj = MatrixXd::Constant(3,4,0.0);
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
         return Hj;
	}
	
    // compute jacobian matrix for linearizing radar measurement
    Hj << px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px,2)+pow(py,2)), 0, 0,
        -(py/(pow(px,2)+pow(py,2))),  px/(pow(px,2)+pow(py,2)),0,0,
        py*(vx*py-vy*px)/(pow(px,2)+pow(py,2),1.5), px*(vy*px-vx*py)/(pow(px,2)+pow(py,2),1.5), px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px,2)+pow(py,2));
std::cout << " Finish .. Calculate Jacobian"  << std::endl;
return Hj;
}


VectorXd Tools::FromPolarToCartesian(const VectorXd& x_state){
/**
   * 
   * Convert state from polar coordinate to cartesian.
   * args: state in polar space
   */
VectorXd cartesian_state(4);

double rho = x_state(0);
double pi = x_state(1);
double rhodot = x_state(2);

double px = rho * cos(pi);
double py = rho * sin(pi);
double vx = rhodot * cos(pi);
double vy = rhodot * sin(pi);

cartesian_state << px, py, vx, vy;

return cartesian_state;

}


VectorXd Tools::FromCartesianToPolar(const VectorXd& x_state){
/**
   * 
   * Convert state from cartesian  coordinate to polar.
   * args: state in cartesian space
   */
VectorXd polar_state(3);

double px = x_state(0);
double py = x_state(1);
double vx = x_state(2);
double vy = x_state(3);

double rho = sqrt(pow(px,2)+pow(py,2));
double pi = atan2(py, px);

// Avoid division by zero
if(rho < 0.000001){
  rho = 0.000001;
}

double rhodot = (px * vx + py * vy) / rho;



polar_state << rho, pi, rhodot;

return polar_state;


}

