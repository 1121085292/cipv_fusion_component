#include "cipv_fusion_component/src/sensor_manager.h"
#include "sensor_manager.h"

SensorManager::SensorManager(RadarData& rr, ModelV2& camera, CarState& car) {
    log_mono_time_["radar"] = rr.can_mono_time();
    log_mono_time_["camera"] = camera.timestamp_eof();
    log_mono_time_["carState"] = car.can_mono_time();

}
bool SensorManager::UpdateSensorData(const std::string &sensorName)
{
    return log_mono_time_[sensorName];
}
