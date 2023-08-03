#include "cipv_fusion_component/src/radard.h"
#include "radard.h"

RadarD::RadarD(double radar_ts, int delay) : kalman_params(radar_ts) {
    current_time = 0.0;

    // 假设 tracks 是一个嵌套的 map
    std::map<int, std::map<std::string, double>> tracks;

    // v_ego 初始化为 0.0
    v_ego = 0.0;

    // 使用 std::deque 作为 v_ego_hist 来存储历史记录
    v_ego_hist = std::deque<double>(delay + 1, 0.0);

    ready = false;
}

void RadarD::Update(SensorManager &sm, RadarData &rr, bool enable_lead) {
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

    if (log_mono_time_["carState"]) {
        v_ego = rr["carState"].vEgo;
        v_ego_hist.push_back(v_ego);
    }

    // 检查 modelV2 数据是否更新
    if (log_mono_time_["modelV2"]) {
        ready = true;
    }
}