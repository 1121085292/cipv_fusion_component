#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cipv_fusion_component/proto/camera_detection.pb.h"

using cipv_fusion_component::proto::LeadsV3;

class CameraPm : public apollo::cyber::TimerComponent {
  public:
    bool Init() override;
    bool Proc() override;

  private:
    std::shared_ptr<apollo::cyber::Writer<LeadsV3>> camera_writer_ = nullptr;

};
CYBER_REGISTER_COMPONENT(CameraPm);