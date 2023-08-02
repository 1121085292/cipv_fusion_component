#include "cipv_fusion_component/component/radar_detection_component.h"
using apollo::cyber::ComponentBase;
using cipv_fusion_component::proto::RadarPoint;

bool RadarPm::Init()
{
    time = 123456;
    radar_writer_ = node_->CreateWriter<RadarData>("/canbus/radar_data/");
    return true;
}

bool RadarPm::Proc()
{
    // std::vector<std::shared_ptr<RadarData>> radar;
    auto out_msg = std::make_shared<RadarData>();
    out_msg->set_errors(cipv_fusion_component::proto::Error::CANERROR);
    out_msg->set_can_mono_time(time++);

    std::vector<std::shared_ptr<RadarPoint>> PointsPtr;
    auto point1 = std::make_shared<RadarPoint>();
    // static int i = 0;
    point1->set_track_id(1);
    point1->set_d_rel(10.5);
    point1->set_y_rel(5.2);
    point1->set_v_rel(15.3);
    point1->set_a_rel(2.1);
    point1->set_y_v_rel(6.0);
    point1->set_measured(false);
    PointsPtr.push_back(point1);
    
    auto point2 = std::make_shared<RadarPoint>();
    point2->set_track_id(2);   // Set a different track_id for the second point
    point2->set_d_rel(8.0);    // Example values, modify as needed
    point2->set_y_rel(3.1);
    point2->set_v_rel(12.9);
    point2->set_a_rel(1.7);
    point2->set_y_v_rel(4.2);
    point2->set_measured(true);  // Example values, modify as needed
    PointsPtr.push_back(point2);

    // Add the PointsPtr vector to the RadarData message
    for (const auto& point : PointsPtr) {
        auto new_point = out_msg->add_points();
        new_point->CopyFrom(*point);
    }
    radar_writer_->Write(out_msg);

    return true;
}
