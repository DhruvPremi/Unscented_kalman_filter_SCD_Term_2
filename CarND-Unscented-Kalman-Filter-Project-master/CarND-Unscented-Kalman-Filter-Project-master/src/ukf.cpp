#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ =0.6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.2;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  n_z_ = 3;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_ , 0,
        0, std_laspy_ * std_laspy_;

  H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0;

  R_radar_ = MatrixXd(n_z_, n_z_);
  R_radar_  <<  pow(std_radr_, 2), 0.0, 0.0,
        0.0, pow(std_radphi_, 2), 0.0,
        0.0, 0.0, pow(std_radrd_, 2);

  is_initialized_ = false;
 
  n_x_ = 5;
  n_aug_ = 7;

  lambda_ = 3 - n_aug_;
  time_us_ = 0;

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.0);
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); 
  Xsig_pred_.fill(0.0);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // First Find out if it is a L or R
  // Is it the first time, time stamp has to be changed and the initial state has to be changed
  // Do the prediction with the help of Delta_t, Points must be choossen , Sigma points, Convert them , and then calculate the new transformation of the sigma points
  // P and X will be changed

  //  Then Based on L or R Measurement update should happen and updates to P and X
  cout << "In ProcessMeasurement" << endl;
  if (!is_initialized_) 
  {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF_1: " << endl;
    // cout << meas_package << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
    
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      x_ << meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]),  meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]), 0, 0, 0;
      cout << "UKFa: " << endl;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) 
    {
      /**
      Initialize state.
      */
      x_  << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      cout << "UKFb: " << endl;
    }

    // done initializing, no need to predict or update

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    cout << "Init Done";
    return;
  }

  double delta_t = (double)(meas_package.timestamp_ - time_us_) / 1000000.0;

  cout << "Delta T  ===  " << delta_t << endl;
  cout << "Time Delta _t " << delta_t << endl;
  time_us_ = meas_package.timestamp_;

  cout << "Before Prediction  " << P_ << endl;


  cout << "Going in Prediction" << endl;
  Prediction(delta_t);

  cout << "After Prediction  " << P_ << endl;


  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
  {
    // Radar updates 
    UpdateRadar(meas_package);
    cout << "Radar " << endl;
  } 
  else 
  {
    // Laser updates
    UpdateLidar(meas_package);
    cout << "Lidar " << endl;

  }

    cout << "After Measurement  " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Find Sigma Points
  // Find Transformations of the Sigma Points
  // Calculate its mean and co-variance

  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0); 
  cout << "Prediction 1" << endl;
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;


  cout << "Prediction 2" << endl;
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  cout << "Prediction 3" << endl;
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  cout << "Prediction 4" << endl;
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);
  Xsig_aug.col(0)  = x_aug;
  
  cout << "Prediction 5" << endl;
  int i;
  double angle_temp;

  for (i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1)         =  x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    
    // angle_temp = Xsig_aug.col(i + 1)[3];
    // Xsig_aug.col(i + 1)[3] = atan2(sin(angle_temp) , cos(angle_temp));

    Xsig_aug.col(i + 1 + n_aug_) =  x_aug - sqrt(lambda_ + n_aug_) * L.col(i);

    // angle_temp = Xsig_aug.col(i + 1 + n_aug_)[3];
    // Xsig_aug.col(i + 1 + n_aug_)[3] = atan2(sin(angle_temp) , cos(angle_temp));
  }

  cout << "Prediction 6" << endl;
  for(i=0 ; i < (2*n_aug_ + 1) ; i+=1)
  {
      VectorXd vec = Xsig_aug.col(i);
      VectorXd output = RateOfChangeState((double) vec[2], (double) vec[3], (double) vec[4], (double) vec[5], (double) vec[6], (double) delta_t);
      
      Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_) +  output;

      // angle_temp = Xsig_pred_.col(i)[3];
      // Xsig_pred_.col(i)[3] = atan2(sin(angle_temp) , cos(angle_temp));
  }

  cout << "Prediction 7" << endl;
  for(i = 0; i < (2*n_aug_ + 1); i += 1)
  {
        if(i == 0)
        {
            weights_[0] = lambda_/(lambda_ + n_aug_);
        
            continue;
        }
        weights_[i] = 0.5/(lambda_ + n_aug_);
  }

  cout << "Prediction 8" << endl; 
  VectorXd x_temp = VectorXd(n_x_);
  x_temp.fill(0.0);

  for(i = 0; i < (2*n_aug_ +1); i += 1)
  {
      x_temp = x_temp + (weights_[i] * Xsig_pred_.col(i));
  }

  // x_[3] = atan2(sin(x_[3]), cos(x_[3]));
  // x_[4] = atan2(sin(x_[4]), cos(x_[4]));

  cout << "Prediction 9" << endl;

  MatrixXd P_temp = MatrixXd(n_x_, n_x_);
  P_temp.fill(0.0);
  for(i = 0; i < (2*n_aug_ + 1); i += 1)
  {
      VectorXd temp_mean = Xsig_pred_.col(i) - x_temp;
      temp_mean[3] = atan2(sin(temp_mean[3]), cos(temp_mean[3]));
      P_temp = P_temp + (weights_[i] * temp_mean * temp_mean.transpose());
  }


  x_ = x_temp;
  P_ = P_temp;
  
 cout << "Prediction Done" << endl;

}

VectorXd UKF::RateOfChangeState(double v, double phi, double phi_dot, double v_a , double v_phi, double delta_t)
{
    VectorXd vec;
    vec = VectorXd(5);
    
    cout << "Prediction 6a" << endl;
    double angle_temp;

    if(phi_dot > 0.001)
    {

        vec[0] = ((v/phi_dot) * (sin(phi + (phi_dot *delta_t)) - sin(phi))) + (0.5 * delta_t *delta_t *cos(phi) * v_a);
        vec[1] = ((v/phi_dot) * ((-1 * cos(phi + (phi_dot *delta_t))) + cos(phi))) + (0.5 * delta_t *delta_t *sin(phi) * v_a);
        vec[2] = 0 + (delta_t * v_a);
        
        angle_temp = (phi_dot *delta_t) + (0.5 * delta_t * delta_t *v_phi);
        // vec[3] = atan2(sin(angle_temp) ,cos(angle_temp));
        vec[3] = angle_temp;

        vec[4] = 0 + (delta_t * v_phi);
    

    }
    else
    {
        vec[0] = (v * cos(phi) * delta_t) + (0.5 * delta_t *delta_t *cos(phi) * v_a);
        vec[1] = (v * sin(phi) * delta_t) + (0.5 * delta_t *delta_t *sin(phi) * v_a);
        vec[2] = 0 + delta_t *v_a;
        
        angle_temp = (phi_dot *delta_t) + (0.5 * delta_t * delta_t *v_phi);
        // vec[3] = atan2(sin(angle_temp) ,cos(angle_temp));
        vec[3] = angle_temp;

        vec[4] = 0 + (delta_t * v_phi);
    }
    
    cout << "Prediction 6b" << endl;
    return vec;
    
}
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  MatrixXd H_ = H_laser_;
  MatrixXd R_ = R_laser_;
  VectorXd z = meas_package.raw_measurements_;

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  // x_[3] = atan2(sin(x_[3]), cos(x_[3]));
  // x_[4] = atan2(sin(x_[4]), cos(x_[4]));

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);

  int i;
  
  for(i = 0 ; i < (2 * n_aug_ + 1); i += 1)
  {
      VectorXd temp_vec;
      temp_vec = VectorXd(n_z_);
      temp_vec.fill(0.0);

      VectorXd temp_input = Xsig_pred_.col(i);
      
      temp_vec[0] = sqrt(pow(temp_input[0], 2) + pow(temp_input[1], 2));
      temp_vec[1] = atan2(temp_input[1], temp_input[0]);

      if (fabs(temp_vec[0]) < 0.0001)
      {
        temp_vec[2] = 0.001;
      } 
      else
      {
        temp_vec[2] = ((temp_input[0]*temp_input[2]*cos(temp_input[3])) + (temp_input[1]*temp_input[2]*sin(temp_input[3])))/temp_vec[0];
      }
      
      Zsig.col(i) = temp_vec;
  }


  for(i = 0; i < (2*n_aug_+1); i += 1)
  {
      z_pred = z_pred + (weights_[i] * Zsig.col(i));
  }
  
  z_pred[1] = atan2(sin(z_pred[1]) ,cos(z_pred[1]));
  // z_pred[2] = atan2(sin(z_pred[2]) ,cos(z_pred[2]));

  for(i = 0; i < (2*n_aug_+1); i += 1)
  {
      VectorXd temp_mean = Zsig.col(i) - z_pred;
      temp_mean[1] = atan2(sin(temp_mean[1]), cos(temp_mean[1]));
      S = S + (weights_[i] * temp_mean * temp_mean.transpose());
  }
  
  S = S + R_radar_;


  // Xsig_pred_

  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);

  for(i = 0; i < (2*n_aug_+1); i += 1)
  {
    VectorXd temp_mean = Zsig.col(i) - z_pred;
    temp_mean[1] = atan2(sin(temp_mean[1]), cos(temp_mean[1]));

    VectorXd x_mean = Xsig_pred_.col(i) - x_;
    x_mean[3] = atan2(sin(x_mean[3]), cos(x_mean[3]));

    Tc = Tc + (weights_[i] * x_mean * temp_mean.transpose());
  }

    MatrixXd kalman_gain = Tc * S.inverse();
    
    x_ = x_ + (kalman_gain * (z -  z_pred));
    P_ = P_ - (kalman_gain * S * kalman_gain.transpose());

    x_[3] = atan2(sin(x_[3]), cos(x_[3]));
    // x_[4] = atan2(sin(x_[4]), cos(x_[4]));

}
