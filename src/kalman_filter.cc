#include "cipv_fusion_component/src/kalman_filter.h"

double Interp(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
    if (x <= xs.front()) {
        return ys.front();
    }
    if (x >= xs.back()) {
        return ys.back();
    }

    for (size_t i = 0; i < xs.size() - 1; ++i) {
        if (x >= xs[i] && x <= xs[i + 1]) {
            double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }

    return 0.0; 
}

KalmanFilter::KalmanFilter(double dt) {
    assert(dt > 0.01 && dt < 0.1 && "Radar time step must be between .01s and 0.1s");

    A = {{1.0, dt}, {0.0, 1.0}};
    C = {1.0, 0.0};

    // Assuming dt is between 0.01 and 0.1
    std::vector<double> dts;
    for (int i = 1; i <= 10; ++i) {
        dts.push_back(dt * 0.01 * i);
    }

    std::vector<double> K0 = {0.12288, 0.14557, 0.16523, 0.18282, 0.19887, 0.21372, 0.22761, 0.24069, 0.2531, 0.26491};
    std::vector<double> K1 = {0.29666, 0.29331, 0.29043, 0.28787, 0.28555, 0.28342, 0.28144, 0.27958, 0.27783, 0.27617};

    K = {{Interp(dt, dts, K0)}, {Interp(dt, dts, K1)}};
}


KF1D::KF1D(const std::vector<std::vector<double>>& x0, const std::vector<std::vector<double>>& A,
         const std::vector<double>& C, const std::vector<std::vector<double>>& K){
     x = x0;
    this->A = A;
    this->C = C;
    this->K = K;

    // Precompute A_K = A - K*C
    A_K = A;
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[i].size(); ++j) {
            A_K[i][j] -= K[i][j] * C[j];
        }
    }
}

void KF1D::Update(const std::vector<double>& meas) {
    // Assuming x and meas have compatible dimensions
    for (size_t i = 0; i < A_K.size(); ++i) {
        double new_x_i = 0.0;
        for (size_t j = 0; j < A_K[i].size(); ++j) {
            new_x_i += A_K[i][j] * x[j][0];
        }
        new_x_i += K[i][0] * meas[0];
        x[i][0] = new_x_i;
    }
}

std::vector<std::vector<double>> KF1D::GetX() const {
    return x;
}
