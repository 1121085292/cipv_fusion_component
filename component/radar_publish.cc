#include "cyber/cyber.h"
#include "cipv_fusion_component/proto/radar_detection.pb.h"

using cipv_fusion_component::proto::RadarState;

int main(int argc, char const *argv[])
{
    // 1.init
    apollo::cyber::Init(argv[0]);
    AINFO << "radar talker node";
    // 2.create node
    auto radar_detection_node = apollo::cyber::CreateNode("radar_node");
    // 3.create publish
    auto radar_detection = radar_detection_node->CreateWriter<RadarState>("/perception/radar_state");
    // 4.organize publish data
    // pubilsh rate
    apollo::cyber::Rate rate(0.5);

    while (apollo::cyber::OK())
    {   
        // organize data
        cipv_fusion_component::proto::LeadData& radar_obj;
        radar_obj.set_d_rel(20.4);
        radar_obj.set_y_rel(0.5);
        auto radar_obstacle = std::make_shared<RadarState>();
        radar_obstacle->set_md_mono_time(123456);
        radar_obstacle->set_car_state_mono_time(123456789);
        radar_obstacle->add_can_mono_times(15231514);
        radar_obstacle->set_allocated_lead_one(radar_obj);
        // publish data
        radar_detection->Write(radar_obstacle);

        rate.Sleep();
    }
    // free 
    apollo::cyber::WaitForShutdown();
    return 0;
}