#include "cipv_fusion_component/component/car_state_component.h"
using apollo::cyber::ComponentBase;

bool CarPm::Init()
{   
    speed = 0.0f;
    car_writer_ = node_->CreateWriter<CarState>("/canbus/car_data/");
    return true;
}

bool CarPm::Proc()
{
    auto out_msg = std::make_shared<CarState>();
    out_msg->set_v_ego(100);
    static uint64 can_mono_time = 345678;
    out_msg->set_can_mono_time(can_mono_time++);
    car_writer_->Write(out_msg);

    return true;
}
