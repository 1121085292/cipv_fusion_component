#include "cipv_fusion_component/src/radard.h"

RadarD::RadarD(double radar_ts, int delay) : kalman_params(radar_ts) {
    current_time = 0.0;

    // // 假设 tracks 是一个嵌套的 map
    // std::map<int, std::map<std::string, double>> tracks;

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
    // 创建unordered_map来存储Radar数据点
    std::unordered_map<int, RadarPoint> ar_pts;
   // 遍历RadarData中的RadarPoint，并存储到unordered_map中
    for (const auto& pt : rr.points()) {
        ar_pts[pt.track_id()] = pt;
    }

    // 移除原数据中缺失的数据点
    for (auto it = tracks.begin(); it != tracks.end();) {
        int trackId = it->first;
        if (ar_pts.find(trackId) == ar_pts.end()) {
            // 当原数据中的trackId在ar_pts中找不到时，从unordered_map中移除该轨迹
            it = tracks.erase(it);
        } 
    }
    // 遍历ar_pts中的数据点并存储到rpt中
    for (const auto& pair : ar_pts) {
        int trackId = pair.first;
        const RadarPoint& rpt = pair.second;
        // 将 v_ego 根据一个固定的时间对齐，以与雷达测量对齐
        float v_lead = rpt.v_rel() + v_ego_hist[0];

        // 创建跟踪对象，如果该跟踪对象不存在或者是一个新的跟踪
        if (tracks.find(trackId) == tracks.end()) {
            tracks[trackId] = Track(v_lead, kalman_params);
        }

        // 更新跟踪对象的信息
        tracks[trackId].Update(rpt.d_rel(), rpt.y_rel(), rpt.v_rel(), v_lead, rpt.measured());
    }
    // track_id容器
    std::vector<int> idens;
    for (const auto& track_pair : tracks) {
        idens.push_back(track_pair.first);
    }
    //track_id排序 
    std::sort(idens.begin(), idens.end());

    std::vector<std::vector<double>> track_pts;
    for (int iden : idens) {
        track_pts.push_back(tracks[iden].GetKeyForCluster());
    }

    // 如果有多个点，则进行聚类
    if (track_pts.size() > 1) {
        std::vector<double> flattened_pts;
        for (const auto& track : track_pts) {
            // 获取 track 的 key for cluster 数据，并将其转换为一维数组
            std::vector<double> key = track.GetKeyForCluster();
            flattened_pts.insert(flattened_pts.end(), key.begin(), key.end());
        }

        // 聚类结果的标签数组
        std::vector<int> cluster_idxs(track_pts.size());

        // 聚类阈值
        double dist = 2.5;

        // 调用聚类函数进行计算
        cluster_points_centroid(static_cast<int>(track_pts.size()), static_cast<int>(track_pts[0].size()), flattened_pts.data(), dist, cluster_idxs.data());

        // 创建 Cluster 对象并进行聚类 std::vector<Cluster> clusters(*std::max_element(cluster_idxs.begin(), cluster_idxs.end()) + 1);
        for (size_t idx = 0; idx < track_pts.size(); ++idx) {
            int cluster_i = cluster_idxs[idx];
            if (clusters[cluster_i] == nullptr) {
                clusters[cluster_i] = Cluster();
            }
            clusters[cluster_i].add(track_pts[idx]);
        }
    } else if (track_pts.size() == 1) {
        // 仅有一个点时的处理
        std::vector<int> cluster_idxs = {0};
        std::vector<Cluster> clusters(1);
        clusters[0].add(track_pts[0]);
    } else {
        std::vector<Cluster> clusters;
    }
}
