#include "cyber/cyber.h"
#include "cipv_fusion_component/proto/car_state.pb.h"

using cipv_fuison_component::proto::LeadsV3;

int main(int argc, char const *argv[])
{
    apollo::cyber::Init(argv[0]);
    AINFO << "camera state----------";

    auto camera_node = apollo::cyber::CreateNode("camera_node");

    auto camera_state = camera_node->CreateWriter<LeadsV3>("/perception/camera");

    apollo::cyber::Rate rate(0.5);

    float i = 0.0f;

    while (apollo::cyber::OK())
    {    
        i += 0.8;
        auto out_msg = std::make_shared<LeadsV3>();
    
        out_msg->add_lead_data_v3();

        out_msg->add_x(i);    
        out_msg->add_x_std(i);    
        out_msg->set_prob(0.5);
    
        camera_state->Write(out_msg);

        rate.Sleep();
    }

    apollo::cyber::WaitForShutdown();
    
    return 0;
}