#pragma once

#include <deque>
#include <map>

#include "cipv_fusion_component/src/kalman_filter.h"
#include "cipv_fusion_component/src/sensor_manager.h"
#include "cipv_fusion_component/src/track.h"
#include "cipv_fusion_component/src/fast_cluster.h"

#include "cipv_fusion_component/src/meta.h"

double laplacian_cdf(double x, double mu, double b);

Cluster* match_vision_to_cluster(double v_ego, const LeadDataV3& lead,
                             const std::vector<std::unique_ptr<Cluster>>& clusters);

std::map<std::string, double> get_lead(double v_ego, bool ready, 
                                    const std::vector<std::unique_ptr<Cluster>>& clusters,
                                    const LeadDataV3& lead_msg, bool low_speed_override = true);

class RadarD {
public:
    RadarD(double radar_ts, int delay = 0);
    void Update(SensorManager& sm, RadarData& rr, bool enable_lead);

private:
    double current_time;
    std::map<int, Track> tracks;
    KalmanFilter kalman_params;
    double v_ego;
    std::deque<double> v_ego_hist;
    bool ready;
};
