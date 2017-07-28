#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
  * predict the state
  */

  x_ = F_ * x_ ; // Considering external motion u as 0
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
  * update the state by using Kalman Filter equations
  */

  VectorXd y = z - (H_ * x_);
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + (K * y);

    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size()); // Identity matrix

  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
  * update the state by using Extended Kalman Filter equations
  */
    VectorXd h_x = VectorXd(3);
    h_x = tools.CalculateHx(x_);

    MatrixXd Hj = MatrixXd(3,4);
    Hj = tools.CalculateJacobian(x_);

    VectorXd y = z - h_x;

    y[1] = atan2(sin(y[1]), cos(y[1]));

    MatrixXd S = Hj * P_ * Hj.transpose() + R_;
    MatrixXd K = P_ * Hj.transpose() * S.inverse();

    x_ = x_ + (K * y);

    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size()); // Identity matrix

    P_ = (I - K * Hj) * P_;
}
