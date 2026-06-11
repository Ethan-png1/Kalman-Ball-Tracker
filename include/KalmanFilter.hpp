#ifndef KalmanFilter_H_
#define KalmanFilter_H_
#include <Eigen/Dense>

class KalmanFilter{
    public:
        KalmanFilter(double dt, double processNoise, double measNoise);
        void init(double px, double py);
        void predict();
        void update(double measX, double measY);
        void setDt(double dt);
        Eigen::Vector4d getState() const;

    private:
        Eigen::Matrix4d F, Q, P;
        Eigen::Matrix <double, 2, 4> H; 
        Eigen::Matrix2d R;
        Eigen::Vector4d x;
         double processNoise_;
};

#endif