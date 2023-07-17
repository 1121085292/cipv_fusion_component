#pragma once

#include<vector>

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "cipv_fusion_component/proto/radar_detection.pb.h"
#include "cipv_fusion_component/proto/car_state.pb.h"
#include "cipv_fusion_component/proto/camera_detection.pb.h"

using apollo::cyber::Component;
using cipv_fusion_component::proto::LeadsV3;
using cipv_fusion_component::proto::RadarState;
using cipv_fusion_component::proto::CarState;

class RadarDetectionComponent : public Component<LeadsV3, RadarState, CarState>{
  public:
    bool Init() override;
    bool Proc(const std::shared_ptr<LeadsV3>& camera_obj,
        const std::shared_ptr<RadarState>& radar_obj,
        const std::shared_ptr<CarState>& car_info) override;

  private:
    // float seq_num;
    //publisher
    std::shared_ptr<apollo::cyber::Writer<RadarState>> fusion_writer_;
    // std::shared_ptr<apollo::cyber::Writer<LeadDataV3>> camera_writer_;
    // std::shared_ptr<apollo::cyber::Writer<CarState>> car_writer_;
    uint64_t radar_can_mono_times;
    // writer
    auto out_msg = std::shared_ptr<RadarState>();
    // cipv_fusion_component::proto::LeadData& radar_obj_info;

};
CYBER_REGISTER_COMPONENT(RadarDetectionComponent);
