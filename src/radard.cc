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

    if (sm.UpdateSensorData("carState")) {
        v_ego = sm["carState"].v_ego();
        v_ego_hist.push_back(v_ego);
    }

    // 检查 modelV2 数据是否更新
    if (sm.UpdateSensorData("modelV2")) {
        ready = true;
    }
   // 遍历RadarData中的RadarPoint，并存储到unordered_map中
    for (const RadarPoint& pt : rr.points()) {
        ar_pts[pt.track_id()] = pt;
    }

    // 移除元数据中缺失的数据点
    for (auto it = tracks.begin(); it != tracks.end();) {
        int trackId = it->first;
        if (ar_pts.find(trackId) == ar_pts.end()) {
            // 当元数据中的trackId在ar_pts中找不到时，从unordered_map中移除该轨迹
            it = tracks.erase(it);
        } else {
            // 否则，检查轨迹信息字典中是否有数据，如果没有，则插入一个空字典
            if (it->second.empty()) {
                it->second = std::map<std::string, Track>();
            }
            // 继续遍历下一个轨迹
            ++it;
        }
    }
    // 遍历ar_pts中的数据点并存储到rpt中
    for (const auto& pair : ar_pts) {
        int trackId = pair.first;
        const RadarPoint& rpt = pair.second;
        
    }

    
}
