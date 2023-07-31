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
    out_msg->mutable_points()->set_track_id(1);
    out_msg->mutable_points()->set_d_rel(10.5);
    out_msg->mutable_points()->set_y_rel(5.2);
    out_msg->mutable_points()->set_v_rel(15.3);
    out_msg->mutable_points()->set_a_rel(2.1);
    out_msg->mutable_points()->set_y_v_rel(6.0);
    out_msg->mutable_points()->set_measured(true);
    
    static uint64_t time = 123456;
    out_msg->set_can_mono_times(time++);
    out_msg->set_errors(cipv_fusion_component::proto::Error::CANERROR);

    radar_writer_->Write(out_msg);

    return true;
}
