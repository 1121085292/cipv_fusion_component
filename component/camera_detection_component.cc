#include "cipv_fusion_component/component/camera_detection_component.h"
using apollo::cyber::ComponentBase;

bool CameraPm::Init()
{
    frame_id_ = 0;
    timestamp_ = 123456789;
    camera_writer_ = node_->CreateWriter<ModelV2>("/perception/camera_data/");
    return true;
}

bool CameraPm::Proc()
{
    auto out_msg = std::make_shared<ModelV2>();
    //write
    out_msg->set_frame_id(frame_id_++);
    out_msg->set_timestamp_eof(timestamp_++);
    camera_writer_->Write(out_msg);

    return true;
}
