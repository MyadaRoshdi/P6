﻿#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

    * predict the state
  */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**

    * update the state by using Kalman Filter equations
  */

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

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  
    * update the state by using Extended Kalman Filter equations
  */
    // To calculate y, we use the h(x') which maps the predicted location x' from cartesian to polar co-ordinates
    //recover state parameters
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];
	
	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px + py*py;
	float rho = sqrt(c1);
	float c3 =  px*vx + py*vy;
	float rhodot = 0;
	float phi = 0;
	VectorXd z_pred = VectorXd(3);

	//check division by zero
	if (fabs(rho) < 0.0001) {
		// so, if rho = 0, use rhodot = 0, to avoid division by zero
	}
	else {

	    rhodot = c3 / rho;
	}

	
	// Using atan(py/px) was not generating accurate angle when the car in the simulator started to turn down of the pow
	// Another thing to mention is that atan2 is more stable when computing tangents using an expression like atan(y/x) and x is 0 or close to 0.
	// atan = gives angle value between -90 and 90, while atan2 = gives angle value between - 180 and 180
	phi = atan2(py , px);
	z_pred << rho, phi, rhodot;
	VectorXd y = z - z_pred;

	// y[1] the value φ,must be normalized to be in-between -π and π
	
	if (y(1) < -M_PI) {

		y(1) = y(1) + 2* M_PI;
	}
	
	else if (y(1) > M_PI) {
		y(1) = y(1) - 2* M_PI;

	}
	
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
}
