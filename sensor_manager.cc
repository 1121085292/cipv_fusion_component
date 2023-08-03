#include "cipv_fusion_component/src/sensor_manager.h"
#include "sensor_manager.h"

SensorManager::SensorManager(RadarData& rr, ModelV2& camera, CarState& car) {
    log_mono_time_["radar"] = rr.can_mono_time();
    log_mono_time_["camera"] = camera.timestamp_eof();
    log_mono_time_["carState"] = car.can_mono_time();

}
void SensorManager::UpdateSensorData(const std::string &sensorName, bool isUpdated)
{
    log_mono_time_[sensorName] = isUpdated;
}