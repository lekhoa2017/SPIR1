#include <iostream>
#include <stdexcept>
#include "spir_atnv/kalman.hpp"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init( const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  initialized = true;
}


void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt) {
  Eigen::MatrixXd Atmp = this->A;
  Atmp(0,1) = dt;
  this->A = Atmp;
  this->dt = dt;
  update(y);
}
