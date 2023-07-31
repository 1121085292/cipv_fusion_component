//publish node

#include "cipv_fusion_component/component/cipv_fusion_component.h"
#include "cipv_fusion_component.h"

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
                                const std::shared_ptr<ModelV2>& camera)
{
    car->v_ego();
    auto out_msg = std::make_shared<RadarState>();
    auto time = radar->can_mono_times();
    out_msg->set_can_mono_times(time);

    fusion_writer_->Write(out_msg);
    AINFO << "Enter cipv fusion component, message timestamps:"
        << radar->can_mono_times() << "current timestamps:"
        << std::max(car->can_mono_times(), camera->timestamp_eof());

    bool status = InternalProc(radar, car, camera, out_msg);
    if (status) {
        fusion_writer_->Write(out_msg);
        AINFO << "Send cipv output message.";
    }
    return true;
}

bool CipvFusionComponent::InternalProc(const std::shared_ptr<RadarData>& radar, 
                                    const std::shared_ptr<CarState>& car, 
                                    const std::shared_ptr<ModelV2>& camera, 
                                    std::shared_ptr<RadarState>& out_msg)
{

    return true;
}