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
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
         return Hj;
	}
	
    // compute jacobian matrix for linearizing radar measurement
    Hj << px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px,2)+pow(py,2)), 0, 0,
        -(py/(pow(px,2)+pow(py,2))),  px/(pow(px,2)+pow(py,2)),0,0,
        py_r*(vx*py-vy*px)/(pow(px,2)+pow(py,2),1.5), px*(vy*px-vx*py)/(pow(px,2)+pow(py,2),1.5), px/sqrt(pow(px,2)+pow(py,2)), py/sqrt(pow(px_r,2)+pow(py_r,2));

return Hj;
}
