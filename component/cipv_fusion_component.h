#pragma once

#include<vector>

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "cipv_fusion_component/proto/radar_detection.pb.h"
#include "cipv_fusion_component/proto/car_state.pb.h"
#include "cipv_fusion_component/proto/camera_detection.pb.h"
#include "cipv_fusion_component/proto/cipv_fusion.pb.h"

using apollo::cyber::Component;
using cipv_fusion_component::proto::ModelV2;
using cipv_fusion_component::proto::RadarData;
using cipv_fusion_component::proto::CarState;
using cipv_fusion_component::proto::RadarState;

class CipvFusionComponent : public Component<RadarData, CarState, ModelV2>{
  public:
    bool Init() override;

    bool Proc(const std::shared_ptr<RadarData>& radar,
              const std::shared_ptr<CarState>& car,
              const std::shared_ptr<ModelV2>& camera) override;

  private:
    //publisher
    std::shared_ptr<apollo::cyber::Writer<RadarState>> fusion_writer_;

    bool InternalProc(const std::shared_ptr<RadarData>& radar,
                    const std::shared_ptr<CarState>& car,
                    const std::shared_ptr<ModelV2>& camera,
                    std::shared_ptr<RadarState>& out_msg);

};
CYBER_REGISTER_COMPONENT(CipvFusionComponent);