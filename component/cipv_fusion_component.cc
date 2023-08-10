//publish node

#include "cipv_fusion_component/component/cipv_fusion_component.h"

using apollo::cyber::ComponentBase;

bool CipvFusionComponent::Init()
{   
    // 从底层读数据
    // AINFO << "radard is waiting for CarParams";
    // CarParams carParams = CarParams::from_bytes()
    // 初始化RadarD对象

    // v_ego 初始化为 0.0
    // v_ego = 0.0;
    // // 使用 std::deque 作为 v_ego_hist 来存储历史记录
    // v_ego_hist = std::deque<double>(delay + 1, 0.0);
    // ready = false;
    // enable_lead = true;
    //init publisher
    fusion_writer_ = ComponentBase::node_->CreateWriter<RadarState>("/perception/output/cipv/");
    inner_fusion_writer_ = ComponentBase::node_->CreateWriter<LiveTracks>("/perception/UI/cipv/");
    return true;
}

bool CipvFusionComponent::Proc(const std::shared_ptr<RadarData> &radar,
                                const std::shared_ptr<CarState>& car,
                                const std::shared_ptr<ModelV2>& camera)
{   
    // double max_time = std::max(car->can_mono_time(), camera->timestamp_eof());
    // current_time = 1e-9 * max_time;
    // AINFO << "fusion timestamp: " << current_time;

    // auto out_msg = std::make_shared<RadarState>();

    std::shared_ptr<RadarState> out_msg(new (std::nothrow)
                                                       RadarState);
    std::shared_ptr<LiveTracks> viz_msg(new (std::nothrow)
                                                      LiveTracks);

    std::shared_ptr<RadarD> radard;
    bool status = radard->Update(radar, car, camera, out_msg, viz_msg, enable_lead_);
    // bool status = InternalProc(radar, car, camera, out_msg, viz_msg);
    if (status) {
        fusion_writer_->Write(out_msg);
        AINFO << "Send cipv output message.";
    }
    return true;
}
