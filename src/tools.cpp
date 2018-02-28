#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

#define EPS 0.0001;

Tools::Tools() : mse(4), 
    residual(4),
    rmse(4)
{
  resetRMSE();
}

Tools::~Tools() {}

void Tools::resetRMSE()
{
  mse << 0, 0, 0, 0;
  residual << 0, 0, 0, 0;
  
}

// Calculate the RMSE
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
  float t = estimations.size(); // Current timestep index

  // check the validity of the inputs
  if( t == 0 )
    cout << "Error in CalculateRMSE:  estimations.size() = 0" << endl;
  if( t != ground_truth.size() )
    cout << "Error in CalculateRMSE: sizes of estimation and ground truth do not match" << endl;

  

  mse = mse*(t-1); 


  residual = estimations[t-1] - ground_truth[t-1];
  residual = residual.array()*residual.array();
  mse += residual;
   

  // Calculate the new mean
  mse = mse/estimations.size();

  // Calculate the RMSE
  rmse = mse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
  // Calculate a Jacobian for the transformation from the state vector 
  // px, py, vx, vy to the radar measurement space
  // rho, phi, rhodot.
  

  MatrixXd Hj(3,4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if( px == 0 && py == 0 )
  {
  	px = EPS;
  	py = EPS;
 	//return Hj;
  }

  //compute the Jacobian 
  float rho = sqrt( px*px + py* py );
  float rho2 = rho*rho;
  float rho3 = rho2*rho;
  Hj <<                 px/rho,                    py/rho,      0,      0,
                      -py/rho2,                   px/rho2,      0,      0,
     py*( vx*py - vy*px )/rho3, px*( vy*px - vx*py )/rho3, px/rho, py/rho;

  return Hj;
}
