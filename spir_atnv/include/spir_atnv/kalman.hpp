#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Eigen/Dense>

/** @class kalman.hpp
 *  @brief Kalman fitler
 *  
 *  based on code by Hayk Martirosyan
 *
 *
 *  @author Wenjie Lu
 *  @contact wenjie.lu@uts.edu.au
 *  @date 2nd Sep 2016
 *  @version 1.0.0
 *  @bug Currently the No known bugs.
 *  @todo None
 *
 */

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  /**
  * Create a blank estimator.
  */
  KalmanFilter();



  /**
  * Initialize the filter with a guess for initial states.
  */
  void init( const Eigen::VectorXd& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXd& y);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXd& y, double dt);

  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };

private:

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;


  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};
#endif