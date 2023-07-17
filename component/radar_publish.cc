#include "cyber/cyber.h"
#include "cipv_fusion_component/proto/radar_detection.pb.h"

using cipv_fusion_component::proto::RadarData;

int main(int argc, char const *argv[])
{
    // 1.init
    apollo::cyber::Init(argv[0]);
    AINFO << "radar state-----------";
    // 2.create node
    auto radar_detection_node = apollo::cyber::CreateNode("radar_node");
    // 3.create publish
    auto radar_detection = radar_detection_node->CreateWriter<RadarData>("/perception/radar_data");
    // 4.organize publish data
    // pubilsh rate
    apollo::cyber::Rate rate(0.5);

    auto out_msg = std::make_shared<RadarData>();

    while (apollo::cyber::OK())
    {   
        // organize data
        out_msg->add_can_mono_times(1);
        out_msg->add_can_mono_times(2);

        out_msg->add_errors(Error::FAULT);

        // publish data
        radar_detection->Write(out_msg);

        rate.Sleep();
    }
    // free 
    apollo::cyber::WaitForShutdown();
    return 0;
}
