#pragma once

#include<vector>
#include<deque>

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "cipv_fusion_component/src/radard.h"

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

    bool InternalProc(const std::shared_ptr<RadarData>& radar,
                    const std::shared_ptr<CarState>& car,
                    const std::shared_ptr<ModelV2>& camera,
                    std::shared_ptr<RadarState>& out_msg);

    double current_time;
    double v_ego;
    std::deque<double> v_ego_hist;
    bool ready;
    
};
CYBER_REGISTER_COMPONENT(CipvFusionComponent);
