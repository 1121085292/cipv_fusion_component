#include "cipv_fusion_component/component/radar_detection_component.h"
using apollo::cyber::ComponentBase;

bool RadarPm::Init()
{
    radar_writer_ = node_->CreateWriter<RadarData>("/canbus/radar_data/");
    return true;
}

bool RadarPm::Proc()
{
    auto out_msg = std::make_shared<RadarData>();
    static uint64_t time = 0;
    
    out_msg->set_can_mono_times(time++);
    radar_writer_->Write(out_msg);

    return true;
}
