#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cipv_fusion_component/proto/radar_detection.pb.h"

using cipv_fusion_component::proto::RadarData;

class RadarPm : public apollo::cyber::TimerComponent {
  public:
    bool Init() override;
    bool Proc() override;

  private:
    std::shared_ptr<apollo::cyber::Writer<RadarData>> radar_writer_ = nullptr;

    static uint64_t time;
};
CYBER_REGISTER_COMPONENT(RadarPm);