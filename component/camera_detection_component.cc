#include "cipv_fusion_component/component/camera_detection_component.h"
using apollo::cyber::ComponentBase;
using cipv_fusion_component::proto::LeadDataV3;

bool CameraPm::Init()
{
    camera_writer_ = node_->CreateWriter<ModelV2>("/perception/camera_data/");
    return true;
}

bool CameraPm::Proc()
{
    auto out_msg = std::make_shared<ModelV2>();
    //write
    static uint32_t frame_id_ = 0;
    static uint64_t timestamp_ = 234567;
    out_msg->set_frame_id(frame_id_++);
    out_msg->set_timestamp_eof(timestamp_++);
    // leadDataV3(0s，2s，4s)
    std::vector<std::shared_ptr<LeadDataV3>> CameraPtr;
    auto leadsV3_1 = std::make_shared<LeadDataV3>();
    leadsV3_1->set_prob(0.8);
    leadsV3_1->set_prob_time(0);
    leadsV3_1->set_t(0);
    leadsV3_1->set_x(10.2);
    leadsV3_1->set_x_std(0.8);
    leadsV3_1->set_y(5.2);
    leadsV3_1->set_y_std(0.9);
    leadsV3_1->set_v(15.3);
    leadsV3_1->set_v_std(0.77);
    leadsV3_1->set_a(2.1);
    leadsV3_1->set_a_std(0.6);
    CameraPtr.push_back(leadsV3_1);

    auto leadsV3_2 = std::make_shared<LeadDataV3>();
    leadsV3_2->set_prob(0.8);
    leadsV3_2->set_prob_time(2);
    leadsV3_2->set_t(2);
    leadsV3_2->set_x(10.2);
    leadsV3_2->set_x_std(0.8);
    leadsV3_2->set_y(5.2);
    leadsV3_2->set_y_std(0.9);
    leadsV3_2->set_v(15.3);
    leadsV3_2->set_v_std(0.77);
    leadsV3_2->set_a(2.1);
    leadsV3_2->set_a_std(0.6);
    CameraPtr.push_back(leadsV3_2);

    for (const auto& cam : CameraPtr) {
        auto new_cam = out_msg->add_leadsv3();
        new_cam->CopyFrom(*cam);
    }
    camera_writer_->Write(out_msg);

    return true;
}
