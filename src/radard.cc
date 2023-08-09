#include "cipv_fusion_component/src/radard.h"

double RadarD::laplacian_cdf(double x, double mu, double b) {
    b = std::max(b, 1e-4);
    return std::exp(-std::abs(x - mu) / b);
}

Cluster* RadarD::match_vision_to_cluster(double v_ego, const LeadDataV3& lead, 
                            const std::vector<std::unique_ptr<Cluster>>& clusters) {
    double offset_vision_dist = lead.x() - RADAR_TO_CAMERA;

    auto prob = [&](const Cluster* c) {
        double prob_d = laplacian_cdf(c->dRel(), offset_vision_dist, lead.x_std());
        double prob_y = laplacian_cdf(c->yRel(), -lead.y(), lead.y_std());
        double prob_v = laplacian_cdf(c->vRel() + v_ego, lead.v(), lead.v_std());

        // 这不是完全准确的，但是是一个不错的启发式方法
        return prob_d * prob_y * prob_v;
    };

    auto max_prob_cluster_iter = std::max_element(clusters.begin(), clusters.end(), 
                                [&](const std::unique_ptr<Cluster>& a, const std::unique_ptr<Cluster>& b) {
        return prob(a.get()) < prob(b.get());
    });

    Cluster* cluster = (max_prob_cluster_iter != clusters.end()) ? max_prob_cluster_iter->get() : nullptr;

    // 如果找不到合理匹配，则返回空指针
    // 静止的雷达点可能是误报
    bool dist_sane = std::abs(cluster->dRel() - offset_vision_dist) < std::max((offset_vision_dist) * 0.25, 5.0);
    bool vel_sane = (std::abs(cluster->vRel() + v_ego - lead.v()) < 10) || (v_ego + cluster->vRel() > 3);
    if (dist_sane && vel_sane) {
        return cluster;
    } else {
        return nullptr;
    }
}

std::map<std::string, float> RadarD::get_lead(double v_ego, bool ready, 
                                    const std::vector<std::unique_ptr<Cluster>>& clusters,
                                    const LeadDataV3& lead_msg, bool low_speed_override = true) {
    std::map<std::string, float> lead_dict;
    lead_dict["status"] = false;

    Cluster* cluster = nullptr;
    if (!clusters.empty() && ready && lead_msg.prob() > 0.5) {
        cluster = match_vision_to_cluster(v_ego, lead_msg, clusters);
    }

    if (cluster != nullptr) {
        lead_dict = cluster->get_RadarState(lead_msg.prob());
    } else if (cluster == nullptr && ready && lead_msg.prob() > 0.5) {
        Cluster temp_cluster; // 创建一个临时 Cluster 对象
        lead_dict = temp_cluster.get_RadarState_from_vision(lead_msg, v_ego);
    }

    if (low_speed_override) {
        std::vector<Cluster*> low_speed_clusters;
        for (const auto& c : clusters) {
            if (c->potential_low_speed_lead(v_ego)) {
                low_speed_clusters.push_back(c.get());
            }
        }
        if (!low_speed_clusters.empty()) {
            Cluster* closest_cluster = *std::min_element(low_speed_clusters.begin(), low_speed_clusters.end(), 
                                    [](const Cluster* a, const Cluster* b) {
                return a->dRel() < b->dRel();
            });
            if ((!lead_dict["status"]) || (closest_cluster->dRel() < lead_dict["dRel"])) {
                lead_dict = closest_cluster->get_RadarState(lead_msg.prob());
            }
        }
    }
    return lead_dict;
}

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

bool RadarD::Update(const std::shared_ptr<RadarData>& rr, 
                    const std::shared_ptr<CarState>& car, 
                    const std::shared_ptr<ModelV2>& camera, 
                    std::shared_ptr<RadarState>& out_msg,
                    std::shared_ptr<LiveTracks>& viz_msg,
                    bool enable_lead) {
    // 创建unordered_map来存储Radar数据点
    std::unordered_map<int, RadarPoint> ar_pts;
   // 遍历RadarData中的RadarPoint，并存储到unordered_map中
    for (const auto& pt : rr->points()) {
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
    // If we have multiple points, cluster them
    std::vector<int> cluster_idxs = cluster_points_centroid(track_pts, 2.5); // 计算聚类
    int max_cluster_idx = *std::max_element(cluster_idxs.begin(), cluster_idxs.end());
    std::vector<std::unique_ptr<Cluster>> clusters(max_cluster_idx + 1);
    if (track_pts.size() > 1) {
        for (int idx = 0; idx < track_pts.size(); ++idx) {
            int cluster_i = cluster_idxs[idx];
            if (clusters[cluster_i] == nullptr) {
                clusters[cluster_i] = std::make_unique<Cluster>();
            }
            clusters[cluster_i]->add(tracks[idens[idx]]);
        }
    } else if (track_pts.size() == 1) {
        std::vector<int> cluster_idxs = {0};
        std::vector<Cluster> clusters(1);
        clusters[0].add(tracks[idens[0]]);
    } else {
        std::vector<Cluster> clusters; // 此处clusters为空，表示没有聚类
    }

    // 如果是一个新的数据点，将加速度重置为聚类中的其余轨迹的平均值
    for (int idx = 0; idx < track_pts.size(); ++idx) {
        if (tracks[idens[idx]].Getcnt() <= 1) {
            double aLeadK = clusters[cluster_idxs[idx]]->aLeadK();
            double aLeadTau = clusters[cluster_idxs[idx]]->aLeadTau();
            tracks[idens[idx]].ResetaLead(aLeadK, aLeadTau);
        }
    }
    std::map<std::string, float> radarState;
    if (enable_lead) {
        if (camera->leadsv3().size() > 1) {
            radarState.mutable_lead_one() = get_lead(v_ego, ready, clusters, camera->leadsv3(0), true);
            out_msg->mutable_lead_two() = get_lead(v_ego, ready, clusters, camera->leadsv3(1), false);
        }
    }
    return dat;
}
