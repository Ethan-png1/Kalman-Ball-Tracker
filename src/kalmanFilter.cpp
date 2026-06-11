#include "KalmanFilter.hpp"
#include <cmath>




KalmanFilter::KalmanFilter(double dt, double processNoise, double measNoise) {
    processNoise_ = processNoise;
    x = Eigen::Vector4d::Zero();
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    R = Eigen::Matrix2d::Identity() * measNoise;
    P = Eigen::Matrix4d::Identity() * 1000;
    F = Eigen::Matrix4d::Identity();
    setDt(dt);   
}
    
void KalmanFilter::init(double px, double py) {
    x << px, py, 0, 0;
    P = Eigen::Matrix4d::Identity() * 1000;
}

void KalmanFilter::predict()
{
    x = F * x;
    P = F * P * F.transpose() + Q;
}

Eigen::Vector4d KalmanFilter::getState() const {
    return x;
}

void KalmanFilter::setDt(double dt) {
    F(0,2) = dt; F(1,3) = dt;
    double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
    Q << dt4/4, 0, dt3/2, 0,
         0, dt4/4, 0, dt3/2,
         dt3/2, 0, dt2, 0,
         0, dt3/2, 0, dt2;
    Q *= processNoise_; 
}

void KalmanFilter::update(double measX, double measY)
{
    Eigen::Vector2d z(measX, measY);
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::Matrix4d::Identity() - K * H) * P;
}

