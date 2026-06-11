#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>
#include "KalmanFilter.hpp"

const int    WIDTH         = 960;
const int    HEIGHT        = 600;
const double DT            = 1.0 / 60.0;
const double BALL_RADIUS   = 12.0;
const double MEAS_STD      = 10.0;  // px of Gaussian noise on each measurement
const double ACCEL_STD     = 250.0; // px/s^2 random acceleration jitter on the ball
const double DROPOUT_PROB  = 0.15;  // chance per frame the "detector" misses the ball
const double PROCESS_ACCEL_STD = 400.0; // px/s^2
const double LOOKAHEAD_S   = 0.25;  // how far ahead to draw the prediction

int main() {
    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<double> measNoise(0.0, MEAS_STD);
    std::normal_distribution<double> accelNoise(0.0, ACCEL_STD);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // true ball state
    double px = WIDTH / 2.0, py = HEIGHT / 2.0;
    std::uniform_real_distribution<double> speed(250.0, 450.0);
    std::uniform_real_distribution<double> angle(0.0, 2.0 * CV_PI);
    double a = angle(rng), s = speed(rng);
    double vx = s * std::cos(a), vy = s * std::sin(a);

    KalmanFilter kf(DT, PROCESS_ACCEL_STD * PROCESS_ACCEL_STD, MEAS_STD * MEAS_STD);
    bool initialized = false;

    cv::Mat canvas(HEIGHT, WIDTH, CV_8UC3);
    while (true) {
        // --- simulate the ball: random acceleration + wall bounces ---
        vx += accelNoise(rng) * DT;
        vy += accelNoise(rng) * DT;
        px += vx * DT;
        py += vy * DT;
        if (px < BALL_RADIUS)          { px = BALL_RADIUS;          vx = -vx; }
        if (px > WIDTH - BALL_RADIUS)  { px = WIDTH - BALL_RADIUS;  vx = -vx; }
        if (py < BALL_RADIUS)          { py = BALL_RADIUS;          vy = -vy; }
        if (py > HEIGHT - BALL_RADIUS) { py = HEIGHT - BALL_RADIUS; vy = -vy; }

        // --- noisy measurement, with occasional dropouts ---
        bool measured = uniform(rng) >= DROPOUT_PROB;
        cv::Point2d meas(px + measNoise(rng), py + measNoise(rng));

        // --- Kalman filter ---
        if (initialized) kf.predict();
        if (measured) {
            if (!initialized) {
                kf.init(meas.x, meas.y);
                initialized = true;
            } else {
                kf.update(meas.x, meas.y);
            }
        }

        // --- draw ---
        canvas.setTo(cv::Scalar(30, 30, 30));
        cv::circle(canvas, cv::Point2d(px, py), (int)BALL_RADIUS,
                   cv::Scalar(255, 255, 255), -1); // true ball

        if (measured) {
            cv::drawMarker(canvas, meas, cv::Scalar(0, 255, 0),
                           cv::MARKER_CROSS, 14, 2); // noisy measurement
        } else {
            cv::putText(canvas, "MEASUREMENT LOST", cv::Point(WIDTH / 2 - 110, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
        }

        if (initialized) {
            Eigen::Vector4d st = kf.getState();
            cv::Point est((int)st[0], (int)st[1]);
            cv::circle(canvas, est, (int)BALL_RADIUS + 6, cv::Scalar(0, 0, 255), 2);

            cv::Point future((int)(st[0] + st[2] * LOOKAHEAD_S),
                             (int)(st[1] + st[3] * LOOKAHEAD_S));
            cv::arrowedLine(canvas, est, future, cv::Scalar(0, 255, 255), 2);
            cv::circle(canvas, future, 8, cv::Scalar(0, 255, 255), 2);
        }

        cv::putText(canvas, "white: true ball   green: measurement   red: KF estimate   yellow: +250ms prediction",
                    cv::Point(10, HEIGHT - 12), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(200, 200, 200), 1);

        cv::imshow("Kalman Ball Tracker", canvas);
        if (cv::waitKey(16) == 27) break; // ~60 fps, ESC to quit
     }

    return 0;
}
