#pragma once

#include <deque>
#include <map>

#include "cipv_fusion_component/src/kalman_filter.h"
#include "cipv_fusion_component/src/sensor_manager.h"

#include "cipv_fusion_component/src/meta.h"

class Track
{
private:
    /* data */
public:
    Track(/* args */);
    ~Track();
};

Track::Track(/* args */)
{
}

Track::~Track()
{
}


class RadarD {
public:
    RadarD(double radar_ts, int delay = 0);
    void Update(SensorManager& sm, RadarData& rr, bool enable_lead);

private:
    double current_time;
    std::map<int, std::map<std::string, Track>> tracks;
    KalmanFilter kalman_params;
    double v_ego;
    std::deque<double> v_ego_hist;
    bool ready;

    // 创建unordered_map来存储Radar数据点
    std::unordered_map<int, RadarPoint> ar_pts;
};
