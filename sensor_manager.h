#pragma once

#include <map>
#include "cipv_fusion_component/src/meta.h"

class SensorManager {
  public:
    SensorManager(RadarData& rr, ModelV2& camera, CarState& car);

    using LogMonoTime = std::map<std::string, double>;
    // LogMonoTime 的 getter 函数
    inline const LogMonoTime& GetLogMonoTime() const{
        return log_mono_time_;
    };

    void UpdateSensorData(const std::string& sensorName, bool isUpdated);
  private:
      // LogMonoTime 是一个存储时间戳的数据类型
    LogMonoTime log_mono_time_;


};

