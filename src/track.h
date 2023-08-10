#pragma once

#include <cmath>
#include <set>
#include <map>

#include "cipv_fusion_component/src/kalman_filter.h"
#include "cipv_fusion_component/src/meta.h"

struct Params
{
    float _LEAD_ACCEL_TAU = 1.5f;
    // float RADAR_TO_CENTER = 2.7f;      //  (deprecated) RADAR is ~ 2.7m ahead from center of car
    float RADAR_TO_CAMERA = 1.52f;      //  RADAR is ~ 1.5m ahead from center of mesh frame
    float v_ego_stationary = 4.0f; 
};

class Track {
public:
    Track() = default;
    
    Track(double v_lead, const KalmanFilter& kalman_params);

    void Update(double d_rel, double y_rel, double v_rel, double v_lead, bool measured);

    std::vector<double> GetKeyForCluster();

    void ResetaLead(double aLeadK, double aLeadTau);

    inline const float Getcnt() const { return cnt;}
    inline const float GetdRel() const { return dRel;}
    inline const float GetyRel() const { return yRel;}
    inline const float GetvRel() const { return vRel;}
    inline const float GetaRel() const { return vLead;}
    inline const float GetvLead() const { return aLeadTau;}
    inline const float GetdPath() const { return dPath;}
    inline const float GetvLat() const { return vLat;}
    inline const float GetvLeadK() const { return vLeadK;}
    inline const float GetaLeadK() const { return aLeadK;}
    inline const float GetaLeadTau() const { return aLeadTau;}
    inline const bool GetMeasured() const { return measured;}
    inline const float GetModelProb() const { return model_prob;}
private:
    int cnt;
    std::vector<std::vector<double>> K_A;
    std::vector<double> K_C;
    std::vector<std::vector<double>> K_K;
    KF1D kf;
    float dRel;
    float yRel;
    float vRel;
    float aRel;
    float vLead;
    float dPath;
    float vLat;
    float vLeadK;
    float aLeadK;
    float aLeadTau;
    bool measured;
    float model_prob;
    // Define SPEED and ACCEL constants if needed
    Params params;

};

struct TrackCompare {
    bool operator()(const Track& lhs, const Track& rhs) const {
        return lhs.Getcnt() < rhs.Getcnt();
    }
};

class Cluster {
public:
    Cluster();

    void add(const Track& t);

    float dRel() const;
    float yRel() const;
    float vRel() const;
    float aRel() const;
    float vLead() const;
    float dPath() const;
    float vLat() const;
    float vLeadK() const;
    float aLeadK() const;
    float aLeadTau() const;

    bool measured() const;
    bool is_potential_fcw(float model_prob) const;

    std::map<std::string, float> get_RadarState(float model_prob) const;
    std::map<std::string, float> get_RadarState_from_vision(const LeadDataV3& lead_msg, double v_ego) const;

    std::string to_string() const;

    bool potential_low_speed_lead(double v_ego);

private:
    std::set<Track, TrackCompare> tracks;
    Params params;
};
