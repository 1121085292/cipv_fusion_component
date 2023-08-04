#pragma once

#include <vector>
#include <cassert>

class KalmanFilter {
public:
    KalmanFilter(double dt);

    std::vector<std::vector<double>> KalmanFilter::GetA() const {
        return A;
    }

    std::vector<double> KalmanFilter::GetC() const {
        return C;
    }

    std::vector<std::vector<double>> KalmanFilter::GetK() const {
        return K;
    }

private:
    std::vector<std::vector<double>> A;
    std::vector<double> C;
    std::vector<std::vector<double>> K;
};

class KF1D {
public:
    KF1D(){};
    KF1D(const std::vector<std::vector<double>>& x0, const std::vector<std::vector<double>>& A,
         const std::vector<double>& C, const std::vector<std::vector<double>>& K);

    void Update(const std::vector<double>& meas);

    std::vector<std::vector<double>> GetX() const;

private:
    std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> A;
    std::vector<double> C;
    std::vector<std::vector<double>> K;
    std::vector<std::vector<double>> A_K;
};
