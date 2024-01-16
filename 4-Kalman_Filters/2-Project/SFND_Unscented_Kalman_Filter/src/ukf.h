#ifndef UKF_H
#define UKF_H

#include <iostream>
#include <stdexcept>

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Creates the augmented state covariance matrix in the prediction step
   */
  Eigen::MatrixXd CreationAugmentedSigmaPoints();

  /**
   * Predicts new sigma points using as input the augmented sigma points into the CTRV
   * (Constant Turn Rate and Velocity Magnitude Model function)
   * @param Xsig_aug  The augmented sigma points
   * @param delta_t   Time difference in seconds 
   */
  void SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, double delta_t);

  /**
   * Predicts new mean and covariance matrix, using as input
   * the new sigma points stored inside ukf class 
   */
  void PredictMeanAndCovariance(); 

  /**
   * Predicts mean predicted measurement and covariance matrix S 
   * from new sigma points
   * @param z_pred    The mean predicted measurement (out value)
   * @param S         The covariance matrix (out value) 
   * @param Zsig      The matrix for sigma points in measurement space (out value)
   */
  void PredictRadarMeasurement(Eigen::VectorXd &z_pred, Eigen::MatrixXd &S, Eigen::MatrixXd &Zsig); 
  
  /**
   * Predicts new state vector X values and also the new state covariance matrix P
   * @param z_pred    The mean predicted measurement
   * @param S         The covariance matrix
   * @param Zsig      The matrix for sigma points in measurement space
   * @param z         The raw measurements from the radar sensor
   * @param n_z       Number of elements in the vector z
   */
  void FinalUpdateStepRadar(const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &S, const Eigen::MatrixXd &Zsig, const Eigen::VectorXd &z, const int n_z);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
};

#endif  // UKF_H