#include "cipv_fusion_component/src/track.h"

Track::Track(double v_lead, const KalmanFilter& kalman_params) {
    cnt = 0;
    aLeadTau = params._LEAD_ACCEL_TAU; // Assuming _LEAD_ACCEL_TAU is defined and set to 1.0

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

Cluster::Cluster() {
    // 初始化
}

void Cluster::add(const Track& t) {
    tracks.insert(t);
}

float Cluster::dRel() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetdRel();
    }
    return sum / tracks.size();
}

float Cluster::yRel() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetyRel();
    }
    return sum / tracks.size();
}

float Cluster::vRel() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetvRel();
    }
    return sum / tracks.size();
}

float Cluster::aRel() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetaRel();
    }
    return sum / tracks.size();
}
float Cluster::vLead() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetvLead();
    }
    return sum / tracks.size();
}

float Cluster::dPath() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetdPath();
    }
    return sum / tracks.size();
}

float Cluster::vLat() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetvLat();
    }
    return sum / tracks.size();
}

float Cluster::vLeadK() const {
    float sum = 0.0;
    for (const auto& t : tracks) {
        sum += t.GetvLeadK();
    }
    return sum / tracks.size();
}

float Cluster::aLeadK() const {
    float sum = 0.0;
    int count = 0;
    for (const auto& t : tracks) {
        if (t.Getcnt() > 1) {
            sum += t.GetaLeadK();
            count++;
        }
    }
    return count > 0 ? sum / count : 0.0;
}

float Cluster::aLeadTau() const {
    float sum = 0.0;
    int count = 0;
    for (const auto& t : tracks) {
        if (t.Getcnt() > 1) {
            sum += t.GetaLeadTau();
            count++;
        }
    }
    return count > 0 ? sum / count : params._LEAD_ACCEL_TAU;
}

bool Cluster::measured() const {
    for (const auto& t : tracks) {
        if (t.GetMeasured()) {
            return true;
        }
    }
    return false;
}

bool Cluster::is_potential_fcw(float model_prob) const {
    return model_prob > 0.9;
}

std::map<std::string, float> Cluster::get_RadarState(float model_prob) const {
    std::map<std::string, float> radar_state;
    radar_state["dRel"] = dRel();
    radar_state["yRel"] = yRel();
    radar_state["vRel"] = vRel();
    radar_state["vLead"] = vLead();
    radar_state["vLeadK"] = vLeadK();
    radar_state["aLeadK"] = aLeadK();
    radar_state["aLeadTau"] = aLeadTau();
    radar_state["status"] = true;
    radar_state["fcw"] = is_potential_fcw(model_prob);
    radar_state["modelProb"] = model_prob;
    radar_state["radar"] = true;
    return radar_state;
}

std::map<std::string, float> Cluster::get_RadarState_from_vision(const LeadDataV3& lead_msg, double v_ego) const {
    std::map<std::string, float> radar_state;
    radar_state["dRel"] = lead_msg.x() - params.RADAR_TO_CAMERA;
    radar_state["yRel"] = -lead_msg.y();
    radar_state["vRel"] = lead_msg.v() - v_ego;
    radar_state["vLead"] = lead_msg.v();
    radar_state["vLeadK"] = lead_msg.v();
    radar_state["aLeadK"] = 0.0;
    radar_state["aLeadTau"] = params._LEAD_ACCEL_TAU;
    radar_state["fcw"] = false;
    radar_state["modelProb"] = lead_msg.prob();
    radar_state["radar"] = false;
    radar_state["status"] = true;
    return radar_state;
}

std::string Cluster::to_string() const {
    // 构建描述 Cluster 的字符串
    return "x: " + std::to_string(dRel()) + "  y: " + std::to_string(yRel()) + "  v: " + std::to_string(vRel()) + "  a: " + std::to_string(aLeadK());
}

bool Cluster::potential_low_speed_lead(double v_ego) {
    return std::abs(yRel()) < 1.5 && v_ego < params.v_ego_stationary && dRel() < 25;
}
