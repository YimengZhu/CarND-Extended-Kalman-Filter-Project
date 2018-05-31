#include "kalman_filter.h"
#include "iostream"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(4,4) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];

  if(px == 0 && py == 0) {
    cout << "px or py is 0, can't convert to radar velocity." << endl;
    return;
  }

  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho_v = (px * vx + py * vy) / rho;

  VectorXd radar_state = VectorXd(3);
  radar_state << rho, phi, rho_v;

  VectorXd y = z - radar_state;
  if(y(1) > M_PI) {
    y(1) -= 2 * M_PI;
  } else if(y(1) < -M_PI) {
    y(1) += 2 * M_PI;
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(4,4) - K * H_) * P_;  
}
