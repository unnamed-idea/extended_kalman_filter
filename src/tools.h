#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {


  VectorXd mse;
  VectorXd residual;
  VectorXd rmse;

  // reset rmse
  void resetRMSE();

public:
  Tools();
  virtual ~Tools();

  // Method to return running root mean standard error
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  // Method to calculate Jacobian for radar measurements
  MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif /* TOOLS_H_ */
