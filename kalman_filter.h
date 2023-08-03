#pragma once

#include <vector>
#include <cassert>

class KalmanFilter {
public:
    KalmanFilter(double dt);

private:
    std::vector<std::vector<double>> A;
    std::vector<double> C;
    std::vector<std::vector<double>> K;
};