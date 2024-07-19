#include "KalmanFilter.hpp"
#include <sstream>
#include <string>
#include <stdexcept>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(VectorXd x, MatrixXd P, MatrixXd T, MatrixXd G, MatrixXd Q, MatrixXd H, MatrixXd R) : x{x}, P{P}, T{T}, G{G}, Q{Q}, H{H}, R{R} {
    // Use the size of the state vector to determine the required dimensions for the identity matrix 
    int n = x.rows();
    I = Eigen::MatrixXd::Identity(n, n);

    // Initialise predicted state and covariance with the given initial values
    x_ = x;
    P_ = P;

    // Initialise Kalman gain matrix
    updateKalmanGain();
}

void KalmanFilter::updateKalmanGain() {
    K = P_ * H.transpose() * (H * P_* H.transpose() + R).inverse();
}

void KalmanFilter::updateStateEstimate(VectorXd u, VectorXd z) {
    // Predict state and covariance
    x_ = T*x + G*u;
    P_ = T*P*T.transpose() + Q; 
    updateKalmanGain();

    // Estimate state and covariance using the Kalman-weighted average of the prediction and measurement
    x = x_ + K * (z - H * x_);
    P = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
}

VectorXd KalmanFilter::getState() {
    return x;
}

MatrixXd KalmanFilter::getCovariance() {
    return P;
}

MatrixXd KalmanFilter::getKalmanGain() {
    return K;
}
