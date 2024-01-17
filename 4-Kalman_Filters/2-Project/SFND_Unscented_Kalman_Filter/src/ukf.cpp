#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  // State dimension
  n_x_ = 5;
  // Augmented state dimension
  n_aug_ = n_x_ + 2;
  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);;
  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  // create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0f);
  // create vector for predicted state
  x_ = VectorXd(n_x_);
  x_.fill(0.0f);
  // create covariance matrix for prediction
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0f);
  // time when the state is true, in us
  time_us_ = 0.0;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  double dt;
  if (is_initialized_ == false) {
    // covariance matrix initialization
    P_ = MatrixXd::Identity(5,5);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // initialization of state vector [pos1 pos2 vel_abs yaw_angle yaw_rate]
      double px, py;
      px = meas_package.raw_measurements_(0, 0); // first element of the vector
      py = meas_package.raw_measurements_(1, 0); // second element of the vector
      x_ << px, py, 0, 0, 0; // the last 3 parameters are unknown in the first reading 
      P_(0,0) = pow(std_laspx_, 2);
      P_(1,1) = pow(std_laspy_, 2);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho, phi, rhoDot;
      rho = meas_package.raw_measurements_(0, 0); // first element of the vector
      phi = meas_package.raw_measurements_(1, 0); // second element of the vector
      rhoDot = meas_package.raw_measurements_(2, 0); // third element of the vector
      x_ << rho*cos(phi), rho*sin(phi), 0, 0, 0; // the last 3 parameters are unknown in the first reading 
      P_(0,0) = pow(std_radr_, 2);
      P_(1,1) = pow(std_radphi_, 2);
      P_(2,2) = pow(std_radrd_, 2);
    } else {
       throw std::invalid_argument("Invalid sensor type detected.");
    }
    // weights vector initialization
    weights_ = VectorXd(2*n_aug_+1);
    double weight = 0.5/(lambda_+n_aug_);
    weights_(0) = lambda_/(lambda_+n_aug_);
    for (int i = 1; i < 2*n_aug_+1; ++i) {  
      weights_(i) = weight;
    }
    // initialization of time_us_
    time_us_ = meas_package.timestamp_;
    // change flag is_initialized 
    is_initialized_ = true;
  }
  // execution of the prediction step
  dt = (meas_package.timestamp_ - time_us_)/pow(10,6); // conversion from us to s
  // update of time_us for the next call
  time_us_ = meas_package.timestamp_;
  UKF::Prediction(dt);
  // execution of the update step
  (meas_package.sensor_type_ == MeasurementPackage::LASER) ? UKF::UpdateLidar(meas_package) : UKF::UpdateRadar(meas_package);

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // calculation sigma point matrix
  MatrixXd Xsig_aug = UKF::CreationAugmentedSigmaPoints();
  // sigma point prediction
  UKF::SigmaPointPrediction(Xsig_aug, delta_t);
  // predict mean and covariance 
  UKF::PredictMeanAndCovariance();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // measurement covariance creation
  MatrixXd R = MatrixXd(2, 2);
  R << pow(std_laspx_, 2), 0,
        0, pow(std_laspy_, 2);
  // measurement matrix creation
  MatrixXd H = MatrixXd(2, n_x_);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;

  // predictions
  VectorXd z_pred = H * x_;
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  const int n_z{3}; // measurement dimension: rho, phi, r_dot
  // create vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0f);
  // create matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0f);
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0.0f);
  VectorXd z = meas_package.raw_measurements_;
  // mean, covariance and sigma points calculation
  PredictRadarMeasurement(z_pred, S, Zsig);
  // final radar update step
  FinalUpdateStepRadar(z_pred, S, Zsig, z, n_z);
}

MatrixXd UKF::CreationAugmentedSigmaPoints() {

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_,  2 * n_aug_ + 1);
 
  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  MatrixXd Q = MatrixXd(2,2);
  Q(0,0) = pow(std_a_, 2);
  Q(1,1) = pow(std_yawdd_, 2);
  P_aug.bottomRightCorner(2,2) = Q;
  
  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  
  // create augmented sigma points
  int i;
  double lambda_plus_na = lambda_+n_aug_;
  MatrixXd AmultiplyFactor = A*sqrt(lambda_plus_na);

  // first column
  Xsig_aug.col(0) = x_aug; 
 
  // x + sqrt(...)
  for (i = 1; i < n_aug_+1; ++i) {
    Xsig_aug.col(i) = x_aug+AmultiplyFactor.col(i-1);
  }
  // x - sqrt(...)
  for(i = n_aug_+1; i < 2*n_aug_+1; ++i) {
    Xsig_aug.col(i) = x_aug-AmultiplyFactor.col(i-n_aug_-1);
  }

  // return result
  return Xsig_aug;
}

void UKF::SigmaPointPrediction(const MatrixXd &Xsig_aug, double delta_t) {

  // clean of previous values from the Xsig_pred matrix
  Xsig_pred_.fill(0.0f);
  double px, py, v, yawnn, yawnnDt, va, vYawnnDt;
  // predict sigma points
  for(int nCols = 0; nCols < 2*n_aug_+1; ++nCols) {
    px = Xsig_aug(0, nCols);
    py = Xsig_aug(1, nCols);
    v = Xsig_aug(2, nCols);
    yawnn = Xsig_aug(3, nCols);
    yawnnDt = Xsig_aug(4, nCols);
    va = Xsig_aug(5, nCols);
    vYawnnDt = Xsig_aug(6, nCols);
    
    if(fabs(yawnnDt) > 0.001) {
      Xsig_pred_(0, nCols) = px + v/yawnnDt * (sin(yawnn+yawnnDt*delta_t) - sin(yawnn) ) + 1/2.0 * pow(delta_t, 2.0) * cos(yawnn) * va;
      Xsig_pred_(1, nCols) = py + v/yawnnDt * (-cos(yawnn+yawnnDt*delta_t) + cos(yawnn) ) + 1/2.0 * pow(delta_t, 2.0) * sin(yawnn) * va;
    } else {
      Xsig_pred_(0, nCols) = px + v * cos(yawnn) * delta_t + 1/2.0 * pow(delta_t, 2.0) * cos(yawnn) * va;
      Xsig_pred_(1, nCols) = py + v * sin(yawnn) * delta_t + 1/2.0 * pow(delta_t, 2.0) * sin(yawnn) * va;
    }
    Xsig_pred_(2, nCols) = v + delta_t*va;
    Xsig_pred_(3, nCols) = yawnn + yawnnDt*delta_t + 1/2.0 * pow(delta_t, 2.0) * vYawnnDt;
    Xsig_pred_(4, nCols) = yawnnDt + delta_t * vYawnnDt;
  }
}

 void UKF::PredictMeanAndCovariance() {
  
  // clean previous values of x_ and P_ 
  x_.fill(0.0f);
  P_.fill(0.0f);
  VectorXd xDiff = VectorXd(n_x_);
  int i, cols;
  // predict state mean x_
  for(cols = 0; cols < 2*n_aug_+1; ++cols) {
    x_+=weights_(cols) * Xsig_pred_.col(cols);
  }
  // predict state covariance matrix P_
  for(cols = 0; cols < 2*n_aug_+1; ++cols) {
    xDiff = Xsig_pred_.col(cols)-x_;
    // angle normalization
    // angle greater than PI
    while (xDiff(3)> M_PI) xDiff(3)-=2.*M_PI;
    // angle less than -PI
    while (xDiff(3)<-M_PI) xDiff(3)+=2.*M_PI;
    
    P_+=weights_(cols)*xDiff*xDiff.transpose();
  }
}

  void UKF::PredictRadarMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig) {
  const int n_z{3}; // measurement dimension: rho, phi, r_dot
  // transform sigma points into measurement space
  int nCols;
  double px, py, v, yawnn, yawnnD;
  for(nCols = 0; nCols < Xsig_pred_.cols(); ++nCols) {
    px = Xsig_pred_(0, nCols);
    py = Xsig_pred_(1, nCols);
    v = Xsig_pred_(2, nCols);
    yawnn = Xsig_pred_(3, nCols);
    yawnnD = Xsig_pred_(4, nCols);
    // ro
    Zsig(0,nCols) = sqrt(pow(px, 2)+pow(py, 2));
    // phi
    Zsig(1, nCols) = atan2(py, px);
    // roDelta
    Zsig(2, nCols) = (px*cos(yawnn)*v+py*sin(yawnn)*v)/sqrt(pow(px, 2)+pow(py, 2));
  }
  // calculate mean predicted measurement
  z_pred.fill(0.0f);
  for(nCols = 0; nCols < Zsig.cols(); ++nCols) {
    z_pred+=weights_(nCols)*Zsig.col(nCols);
  }
  // calculate innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z, n_z);
  VectorXd zDiff;
  R.fill(0.0f);
  R(0,0) = pow(std_radr_, 2);
  R(1,1) = pow(std_radphi_, 2);
  R(2,2) = pow(std_radrd_, 2);
  S.fill(0.0f);
  for(nCols = 0; nCols < Zsig.cols(); ++nCols) {
    zDiff = Zsig.col(nCols)-z_pred;
    // angle normalization
    while (zDiff(1)> M_PI) zDiff(1)-=2.*M_PI;
    while (zDiff(1)<-M_PI) zDiff(1)+=2.*M_PI;
    S+=weights_(nCols)*zDiff*zDiff.transpose();
  }
  // add noise as well 
  S+=R;
} 

void UKF::FinalUpdateStepRadar(const VectorXd &z_pred, const MatrixXd &S, const MatrixXd &Zsig, const VectorXd &z, const int n_z) {
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  int nCols;
  VectorXd xDiff, zDiff;
  Tc.fill(0.0f);
  // cross correlation Tc matrix calculation
  for(nCols = 0; nCols < Xsig_pred_.cols(); ++nCols) {
    xDiff = (Xsig_pred_.col(nCols)-x_);
    zDiff = (Zsig.col(nCols)-z_pred);
    
    // angle normalization
    while (xDiff(3)> M_PI) xDiff(3)-=2.*M_PI;
    while (xDiff(3)<-M_PI) xDiff(3)+=2.*M_PI;
    // angle normalization
    while (zDiff(1)> M_PI) zDiff(1)-=2.*M_PI;
    while (zDiff(1)<-M_PI) zDiff(1)+=2.*M_PI;

    Tc+=weights_(nCols)*xDiff*zDiff.transpose();
  }
  // calculation of Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  // update state mean and covariance matrix
  zDiff = z-z_pred;
  
  // angle normalization
  while (zDiff(1)> M_PI) zDiff(1)-=2.*M_PI;
  while (zDiff(1)<-M_PI) zDiff(1)+=2.*M_PI;
  
  // state vector update
  x_ = x_ + K*zDiff;
  // covariance matrix update
  P_-=K*S*K.transpose();
}
