//publish node

#include "cipv_fusion_component/component/cipv_fusion_component.h"

using apollo::cyber::ComponentBase;

bool CipvFusionComponent::Init()
{   
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
        v_ego_ = car->v_ego();
        v_ego_hist_.push_back(v_ego);
    } else {
        AINFO << "No car state sign";
        return false;
    }

    if(camera){
        ready_ = true;
    } else {
        AINFO << "No camera sign";
        return false;
    }
    current_time_ = std::max(car->can_mono_time(), camera->timestamp_eof());
    auto out_msg = std::make_shared<RadarState>();
    
    AINFO << "Enter cipv fusion component, message timestamps:"
        << radar->can_mono_time() << "current timestamps:"
        << current_time_;

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
    // radar
    for (const auto& pt : radar->points()) {
        // 将每个点的 trackId 作为 key，对应的 PointData 存储为 value
        ar_pts_[pt.track_id] = {pt.d_rel, pt.y_rel, pt.v_rel, pt.measured};
    }
    //  *** remove missing points from meta data ***
    
    return true;
}

double CipvFusionComponent::LaplacianCdf(double x, double mu, double b)
{
    b = std::max(b, 1e-4);
    return std::exp(-std::abs(x - mu)/b);
}
