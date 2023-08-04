#include "cipv_fusion_component/src/track.h"

double Track::_LEAD_ACCEL_TAU = 1.5;

Track::Track(double v_lead, const KalmanFilter& kalman_params) {
    cnt = 0;
    aLeadTau = _LEAD_ACCEL_TAU; // Assuming _LEAD_ACCEL_TAU is defined and set to 1.0

    K_A = kalman_params.GetA();
    K_C = kalman_params.GetC();
    K_K = kalman_params.GetK();

    std::vector<std::vector<double>> x0 = {{v_lead}, {0.0}};
    kf = KF1D(x0, K_A, K_C, K_K);
}

void Track::Update(double d_rel, double y_rel, double v_rel, double v_lead, bool measured) {
    dRel = d_rel;
    yRel = y_rel;
    vRel = v_rel;
    vLead = v_lead;
    this->measured = measured;

    if (cnt > 0) {
        std::vector<double> meas = {v_lead};
        kf.Update(meas);
    }

    vLeadK = kf.GetX()[0][0];
    aLeadK = kf.GetX()[1][0];

    if (std::abs(aLeadK) < 0.5) {
        aLeadTau = 1.0; // Assuming _LEAD_ACCEL_TAU is defined and set to 1.0
    } else {
        aLeadTau *= 0.9;
    }

    cnt += 1;
}

std::vector<double> Track::GetKeyForCluster() {
    std::vector<double> key = {dRel, yRel * 2, vRel};
    return key;
}

void Track::ResetaLead(double aLeadK, double aLeadTau) {
    std::vector<std::vector<double>> x0 = {{vLead}, {aLeadK}};
    kf = KF1D(x0, K_A, K_C, K_K);
    this->aLeadK = aLeadK;
    this->aLeadTau = aLeadTau;
}