#pragma once
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cipv_fusion_component/proto/car_state.pb.h"

using cipv_fusion_component::proto::CarState;

class CarPm : public apollo::cyber::TimerComponent {
  public:
    bool Init() override;
    bool Proc() override;

  private:
    int speed;
    std::shared_ptr<apollo::cyber::Writer<CarState>> car_writer_ = nullptr;

};
CYBER_REGISTER_COMPONENT(CarPm);