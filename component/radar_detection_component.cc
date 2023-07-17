//publish node

#include "cipv_fusion_component/component/radar_detection_component.h"

// using cipv_fusion_component::proto::RadarState;
using apollo::cyber::ComponentBase;

bool RadarDetectionComponent::Init()
{   
    // seq_num = 0.0f;
    //init publisher
    fusion_writer_ = ComponentBase::node_->CreateWriter<RadarState>("/perception/output/cipv");
    // camera_writer_ = node_->CreateWriter<LeadDataV3>("/perception/camera");
    // car_writer_ = node_->CreateWriter<CarState>("/car_state");
    
    return true;
}

bool RadarDetectionComponent::Proc(const std::shared_ptr<LeadsV3>& camera,
                                const std::shared_ptr<RadarState>& radar,
                                const std::shared_ptr<CarState>& car_info)
{       
    // AINFO << "Enter cipv fusion component, message timestamps:"
    //       << radar->md_mono_time() << "current timestamps:"
    //       << 
    bool status = InternalProc(camera, radar, car_info, out_message);
    if (status) {
        fusion_writer_->Write(out_msg);
        AINFO << "Send cipv output message.";
    }
    return status;
}

// template<calss T>
// void ReadMessage(const std::shared_ptr<T>& obj) {
    
// }

bool RadarDetectionComponent::InternalProc(const std::shared_ptr<LeadsV3>& camera,
                                const std::shared_ptr<RadarState>& radar,
                                const std::shared_ptr<CarState>& car_info,
                                std::shared_ptr<RadarState>& out_msg) {
    //1.解析数据
    // camera
    // float prob = camera->prob();
    // std::vector<float> camera_x;
    // int size = camera->x_size();
    // for(int i = 0; i < size; ++i){
    //     float x = camera->x(i);
    //     camera_x.push_back(x);
    // }
    
    // // repeated value
    // int size = radar->can_mono_times_size();
    // for(int i = 0; i < size; ++i){
    //     out_msg->set_can_mono_times(i);
    // }
    // optional value
    uint64_t radar_md_mono_time = radar->md_mono_time();
    uint64_t car_state_mono_time = radar->car_state_mono_time();

    // class  
    // cipv_fusion_component::proto::LeadData& radar_obj;
    // radar_obj.
    // writer
    // auto out_msg = std::shared_ptr<RadarState>();
    out_msg->set_md_mono_time(radar_md_mono_time);
    out_msg->set_car_state_mono_time(car_state_mono_time);

    radar_writer_->Write(out_msg);

    return true;
}
