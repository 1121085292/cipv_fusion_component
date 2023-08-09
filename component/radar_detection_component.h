#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cipv_fusion_component/src/meta.h"

class RadarPm : public apollo::cyber::TimerComponent {
  public:
    bool Init() override;
    bool Proc() override;

  private:
    std::shared_ptr<apollo::cyber::Writer<RadarData>> radar_writer_ = nullptr;

};
CYBER_REGISTER_COMPONENT(RadarPm);
