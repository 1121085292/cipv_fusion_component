//publish node

#include "cipv_fusion_component/component/cipv_fusion_component.h"

using apollo::cyber::ComponentBase;

bool CipvFusionComponent::Init()
{   
    // seq_num = 0.0f;
    current_time = 0;
    v_ego = 0;
    ready = false;
    //init publisher
    fusion_writer_ = ComponentBase::node_->CreateWriter<RadarState>("/perception/output/cipv/");
    inner_fusion_writer_ = ComponentBase::node_->CreateWriter<LiveTracks>("/perception/UI/cipv/");
    return true;
}

bool CipvFusionComponent::Proc(const std::shared_ptr<RadarData> &radar,
                                const std::shared_ptr<CarState>& car,
                                const std::shared_ptr<ModelV2>& camera)
{   
    if(car){
        v_ego = car->v_ego();
        v_ego_hist.push_back(v_ego);
    } else {
        AINFO << "No car state sign";
        return false;
    }

    if(camera){
        ready = true;
    } else {
        AINFO << "No camera sign";
        return false;
    }
    current_time = std::max(car->can_mono_time(), camera->timestamp_eof());
    auto out_msg = std::make_shared<RadarState>();
    
    AINFO << "Enter cipv fusion component, message timestamps:"
        << radar->can_mono_time() << "current timestamps:"
        << current_time;

    bool status = InternalProc(rr, car, camera, out_msg);
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
    // camera

    return true;
}

double CipvFusionComponent::LaplacianCdf(double x, double mu, double b)
{
    b = std::max(b, 1e-4);
    return std::exp(-std::abs(x - mu)/b);
}
