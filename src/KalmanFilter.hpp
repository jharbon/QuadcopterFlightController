#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "ArduinoEigen.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
    private:
    /*
    General n-dimensional case
    n - number of variables comprising the state 
    u - number of inputs 
    m - number of measurements
    */

    VectorXd x;  // Estimate of the system state vector - nx1 
    MatrixXd P;  // Covariance matrix of the system state estimation - nxn 
    MatrixXd K;  // Kalman gain matrix - nxm
    MatrixXd I;  // Identity matrix - nxn

    VectorXd x_;  // Prediction of the system state vector - nx1 
    MatrixXd T;  // State transition matrix - nxn 
    MatrixXd G;  // Control matrix - nxu 

    MatrixXd P_;  // Covariance matrix of the system state prediction - nxn 
    MatrixXd Q;  // Process noise matrix - nxn 

    MatrixXd H;  // Observation matrix - mxn 
    MatrixXd R;  // Measurement noise matrix - mxm 

    void updateKalmanGain();

    public:
    KalmanFilter() {}
    KalmanFilter(VectorXd x, MatrixXd P, MatrixXd T, MatrixXd G, MatrixXd Q, MatrixXd H, MatrixXd R);

    void updateStateEstimate(VectorXd u, VectorXd z);
    VectorXd getState();
    MatrixXd getCovariance();
    MatrixXd getKalmanGain();
};

#endif
