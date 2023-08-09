#pragma once

#include<vector>
#include<deque>
#include <map>

#include "cyber/component/component.h"
#include "cyber/cyber.h"

// #include "cipv_fusion_component/src/kalman_filter.h"
// #include "cipv_fusion_component/src/sensor_manager.h"
// #include "cipv_fusion_component/src/track.h"
// #include "cipv_fusion_component/src/fast_cluster.h"
#include "cipv_fusion_component/src/radard.h"
// #include "cipv_fusion_component/src/meta.h"

using apollo::cyber::Component;

class CipvFusionComponent : public Component<RadarData, CarState, ModelV2>{
  public:
    bool Init() override;

    bool Proc(const std::shared_ptr<RadarData>& radar,
              const std::shared_ptr<CarState>& car,
              const std::shared_ptr<ModelV2>& camera) override;

  private:
    //publisher
    std::shared_ptr<apollo::cyber::Writer<RadarState>> fusion_writer_;
    std::shared_ptr<apollo::cyber::Writer<LiveTracks>> inner_fusion_writer_;

    // bool InternalProc(const std::shared_ptr<RadarData>& rr, 
    //                   const std::shared_ptr<CarState>& car, 
    //                   const std::shared_ptr<ModelV2>& camera, 
    //                   std::shared_ptr<RadarState>& out_msg,
    //                   std::shared_ptr<LiveTracks>& viz_msg);

    // double laplacian_cdf(double x, double mu, double b);

    // Cluster* match_vision_to_cluster(double v_ego, const LeadDataV3& lead,
    //                          const std::vector<std::unique_ptr<Cluster>>& clusters);

    // std::map<std::string, double> get_lead(double v_ego, bool ready, 
    //                                 const std::vector<std::unique_ptr<Cluster>>& clusters,
    //                                 const LeadDataV3& lead_msg, bool low_speed_override = true);

    // double current_time;
    // double v_ego;
    // std::deque<double> v_ego_hist;
    // bool ready;
    // bool enable_lead;
    // std::map<int, Track> tracks;

};
CYBER_REGISTER_COMPONENT(CipvFusionComponent);
