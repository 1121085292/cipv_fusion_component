//publish node

#include "cipv_fusion_component/component/cipv_fusion_component.h"

using apollo::cyber::ComponentBase;

bool CipvFusionComponent::Init()
{   
    // 从底层读数据
    // AINFO << "radard is waiting for CarParams";
    // CarParams carParams = CarParams::from_bytes()
    // 初始化RadarD对象
    //init publisher
    fusion_writer_ = ComponentBase::node_->CreateWriter<RadarState>("/perception/output/cipv/");
    inner_fusion_writer_ = ComponentBase::node_->CreateWriter<LiveTracks>("/perception/UI/cipv/");
    return true;
}

bool CipvFusionComponent::Proc(const std::shared_ptr<RadarData> &radar,
                                const std::shared_ptr<CarState>& car,
                                const std::shared_ptr<ModelV2>& camera)
{   
    SensorManager& sm,
    const SensorManager::LogMonoTime& logMonoTime = sm.GetLogMonoTime();
    // 找到最大的时间戳，并将其转换为秒
    double max_time = 0.0;
    for (const auto& pair : logMonoTime) {
        double timestamp = pair.second;
        if (timestamp > max_time) {
            max_time = timestamp;
        }
    }
    current_time = 1e-9 * max_time;
    AINFO << "fusion timestamp: " << current_time;

    auto out_msg = std::make_shared<RadarState>();
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
