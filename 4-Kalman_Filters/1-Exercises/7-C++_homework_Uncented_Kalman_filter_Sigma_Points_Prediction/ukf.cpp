#include <iostream>
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}


/**
 * Programming assignment functions: 
 */

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // time diff in sec

  /**
   * Student part begin
   */
  Xsig_pred.fill(0.0f);
  double px, py, v, yawnn, yawnnDt, va, vYawnnDt;
  // predict sigma points
  for(int nCols = 0; nCols < Xsig_pred.cols(); ++nCols) {
    px = Xsig_aug(0, nCols);
    py = Xsig_aug(1, nCols);
    v = Xsig_aug(2, nCols);
    yawnn = Xsig_aug(3, nCols);
    yawnnDt = Xsig_aug(4, nCols);
    va = Xsig_aug(5, nCols);
    vYawnnDt = Xsig_aug(6, nCols);
    
    if(yawnnDt != 0) {
      Xsig_pred(0, nCols) = px + v/yawnnDt * (sin(yawnn+yawnnDt*delta_t) - sin(yawnn) ) + 1/2.0 * pow(delta_t, 2.0) * cos(yawnn) * va;
      Xsig_pred(1, nCols) = py + v/yawnnDt * (-cos(yawnn+yawnnDt*delta_t) + cos(yawnn) ) + 1/2.0 * pow(delta_t, 2.0) * sin(yawnn) * va;
    } else {
      Xsig_pred(0, nCols) = px + v * cos(yawnn) * delta_t + 1/2.0 * pow(delta_t, 2.0) * cos(yawnn) * va;
      Xsig_pred(1, nCols) = py + v * sin(yawnn) * delta_t + 1/2.0 * pow(delta_t, 2.0) * sin(yawnn) * va;
    }
      Xsig_pred(2, nCols) = v + delta_t*va;
      Xsig_pred(3, nCols) = yawnn + yawnnDt*delta_t + 1/2.0 * pow(delta_t, 2.0) * vYawnnDt;
      Xsig_pred(4, nCols) = yawnnDt + delta_t * vYawnnDt;
  }
  // avoid division by zero

  // write predicted sigma points into right column

  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;
}