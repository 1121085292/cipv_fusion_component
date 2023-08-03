#include "cipv_fusion_component/src/kalman_filter.h"

double interp(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
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

    std::vector<double> dts;
    std::vector<double> K0;
    std::vector<double> K1;

    for (int i = 1; i <= 10; ++i) {
        dts.push_back(dt * 0.01 * i);
    }

    // 这里初始化 K0 和 K1 列表
    K0 = {0.12288, 0.14557, 0.16523, 0.18282, 0.19887, 0.21372, 0.22761, 0.24069, 0.2531, 0.26491};
    K1 = {0.29666, 0.29331, 0.29043, 0.28787, 0.28555, 0.28342, 0.28144, 0.27958, 0.27783, 0.27617};

    // 使用线性插值计算 K
    A = {{1.0, dt}, {0.0, 1.0}};
    C = {1.0, 0.0};
    K = {{interp(dt, dts, K0)}, {interp(dt, dts, K1)}};
}