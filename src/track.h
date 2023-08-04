#pragma once
#include <vector>
#include <cmath>

#include "cipv_fusion_component/src/kalman_filter.h"
class Track {
public:
    Track(double v_lead, const KalmanFilter& kalman_params);

    void Update(double d_rel, double y_rel, double v_rel, double v_lead, bool measured);

    std::vector<double> GetKeyForCluster();

    void ResetaLead(double aLeadK, double aLeadTau);

private:
    int cnt;
    double aLeadTau;
    std::vector<std::vector<double>> K_A;
    std::vector<double> K_C;
    std::vector<std::vector<double>> K_K;
    KF1D kf;
    double dRel;
    double yRel;
    double vRel;
    double vLead;
    bool measured;
    double vLeadK;
    double aLeadK;

    static double _LEAD_ACCEL_TAU;
    // Define SPEED and ACCEL constants if needed
};