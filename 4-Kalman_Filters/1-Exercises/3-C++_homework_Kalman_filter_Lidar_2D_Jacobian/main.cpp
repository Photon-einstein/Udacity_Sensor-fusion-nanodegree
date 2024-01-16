#include <iostream>
#include <vector>
#include "./../Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  if (px == 0 && py == 0) {
    std::cout<<"Error: Division by zero not allowed.";
    return Hj;
  }
  // compute the Jacobian matrix
  double denominator = pow(px,2)+pow(py,2);
  // 1st row
  Hj(0,0) = px/pow(denominator, 0.5);
  Hj(0,1) = py/pow(denominator, 0.5);
  // 2nd row
  Hj(1,0) = -py/denominator;
  Hj(1,1) = px/denominator;
  // 3rd row
  Hj(2,0) = py*(vx*py-vy*px)/pow(denominator, 1.5);
  Hj(2,1) = px*(vy*px-vx*py)/pow(denominator, 1.5);
  Hj(2,2) = Hj(0,0);
  Hj(2,3) = Hj(0,1);

  return Hj;
}