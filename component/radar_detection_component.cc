//publish node

#include "cipv_fusion_component/component/radar_detection_component.h"

// using cipv_fusion_component::proto::RadarState;
using apollo::cyber::ComponentBase;

bool RadarDetectionComponent::Init()
{   
    // seq_num = 0.0f;
    //init publisher
    writer_ = ComponentBase::node_->CreateWriter<RadarState>("/perception/radar_state");
    camera_writer_ = node_->CreateWriter<LeadDataV3>("/perception/camera");
    car_writer_ = node_->CreateWriter<CarState>("/car_state");
    
    return true;
}

bool RadarDetectionComponent::Proc(const std::shared_ptr<LeadDataV3>& camera_obj,
                                const std::shared_ptr<RadarState>& radar_obj,
                                const std::shared_ptr<CarState>& car_info)
{       
    //read message
    // 1.repeated
    // int size = radar_obj->can_mono_times_size();
    // for(size_t i = 0; i < size; ++i){
    //     radar_can_mono_times = radar_obj->can_mono_times(i);
    // }

    uint64_t radar_md_mono_time = radar_obj->md_mono_time();
    uint64_t car_state_mono_time = radar_obj->car_state_mono_time();

    // //class LeadData
    // uint64_t d = radar_obj_info.d_rel();
    // uint64_t y = radar_obj_info.y_rel();
    float i = 0.8f;
    float j = 50.2f;

    // writer
    auto out_msg1 = std::shared_ptr<RadarState>();
    auto out_msg2 = std::shared_ptr<LeadDataV3>();
    auto out_msg3 = std::shared_ptr<CarState>();
    // out_msg->add_can_mono_times(radar_can_mono_times);
    out_msg1->set_md_mono_time(radar_md_mono_time);
    out_msg1->set_md_mono_time(car_state_mono_time);
    out_msg2->set_prob(i);
    out_msg3->set_v_ego(j);
    // out_msg->mutable_lead_one()->CopyFrom(radar_obj_info);  

    writer_->Write(out_msg1);
    camera_writer_->Write(out_msg2);
    car_writer_->Write(out_msg3);
    return true;
}

