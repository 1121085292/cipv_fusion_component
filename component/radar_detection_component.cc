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
    RadarData& radar;
    radar.set_errors(cipv_fusion_component::proto::Error::CANERROR);
    radar.mutable_points()->set_track_id(1);
    radar.mutable_points()->set_d_rel(10.5);
    radar.mutable_points()->set_y_rel(5.2);
    radar.mutable_points()->set_v_rel(15.3);
    radar.mutable_points()->set_a_rel(2.1);
    radar.mutable_points()->set_y_v_rel(6.0);
    radar.mutable_points()->set_measured(true);
    radar.set_can_mono_times(123456789);

    out_msg->set_can_mono_times(time++);
    radar_writer_->Write(out_msg);

    return true;
}
