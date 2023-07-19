//publish node

#include "cipv_fusion_component/component/cipv_fusion_component.h"

using apollo::cyber::ComponentBase;

bool CipvFusionComponent::Init()
{   
    // seq_num = 0.0f;
    //init publisher
    fusion_writer_ = ComponentBase::node_->CreateWriter<RadarState>("/perception/output/cipv/");
    return true;
}

bool CipvFusionComponent::Proc(const std::shared_ptr<RadarData> &radar,
                                const std::shared_ptr<CarState>& car,
                                const std::shared_ptr<LeadsV3>& camera)
{
    car->v_ego();
    auto out_msg = std::make_shared<RadarState>();
    out_msg->add_can_mono_times(time);

    fusion_writer_->Write(out_msg);
    return true;
}

// bool RadarDetectionComponent::Proc(const std::shared_ptr<LeadsV3>& camera,
//                                 const std::shared_ptr<RadarData>& radar,
//                                 const std::shared_ptr<CarState>& car_info)
// {       
//     // AINFO << "Enter cipv fusion component, message timestamps:"
//     //       << radar->md_mono_time() << "current timestamps:"
//     //       << 
//     auto out_msg = std::make_shared<RadarState>();
//     bool status = InternalProc(camera, radar, car_info, out_msg);
//     if (status) {
//         fusion_writer_->Write(out_msg);
//         AINFO << "Send cipv output message.";
//     }
//     return status;
// }


// bool RadarDetectionComponent::InternalProc(const std::shared_ptr<LeadsV3>& camera,
//                                 const std::shared_ptr<RadarData>& radar,
//                                 const std::shared_ptr<CarState>& car_info,
//                                 std::shared_ptr<RadarState>& out_msg) {

//     uint64_t canmonotime = radar->can_mono_times();
//     out_msg->add_can_mono_times(canmonotime);

//     return true;
// }