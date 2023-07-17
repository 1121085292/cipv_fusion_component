#include "cyber/cyber.h"
#include "cipv_fusion_component/proto/car_state.pb.h"

using cipv_fuison_component::proto::CarState;

int main(int argc, char const *argv[])
{
    apollo::cyber::Init(argv[0]);
    AINFO << "car state";

    auto car_node = apollo::cyber::CreateNode("car_node");

    auto car_state = car_node->CreateWriter<CarState>("/car_state");

    apollo::cyber::Rate rate(0.5);

    float i = 0.0f;

    while (apollo::cyber::OK())
    {   
        i += 0.5;
        auto out_msg = std::make_shared<CarState>();
        out_msg->set_v_ego(i);   
        out_msg->add_can_mono_times(i);
        car_state->Write(out_msg);

        rate.Sleep();
    }

    apollo::cyber::WaitForShutdown();
    
    return 0;
}
