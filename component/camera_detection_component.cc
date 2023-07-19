#include "cipv_fusion_component/component/camera_detection_component.h"
using apollo::cyber::ComponentBase;

bool CameraPm::Init()
{
    camera_writer_ = node_->CreateWriter<LeadsV3>("/perception/camera_data/");
    return true;
}

bool CameraPm::Proc()
{
    auto out_msg = std::make_shared<LeadsV3>();
    //write
    out_msg->set_frame_id(1);
    radar_writer_->Write(out_msg);

    return true;
}
