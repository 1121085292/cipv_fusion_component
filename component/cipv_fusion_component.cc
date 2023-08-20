//publish node

#include "cipv_fusion_component/component/cipv_fusion_component.h"

#include <boost/python.hpp>

using apollo::cyber::ComponentBase;

using namespace boost::python;

bool CipvFusionComponent::Init()
{   
    // 从底层读数据

    //init publisher
    fusion_writer_ = ComponentBase::node_->CreateWriter<RadarState>("/perception/output/cipv/");
    // inner_fusion_writer_ = ComponentBase::node_->CreateWriter<LiveTracks>("/perception/UI/cipv/");
    return true;
}

bool CipvFusionComponent::Proc(const std::shared_ptr<RadarData> &radar,
                                const std::shared_ptr<CarState>& car,
                                const std::shared_ptr<ModelV2>& camera)
{   

    std::shared_ptr<RadarState> out_msg(new (std::nothrow)
                                                       RadarState);
    std::shared_ptr<LiveTracks> viz_msg(new (std::nothrow)
                                                      LiveTracks);

    std::shared_ptr<RadarD> radard = std::make_shared<RadarD>(radar_ts_, 0);
    // bool status = radard->Update(radar, car, camera, out_msg, viz_msg, enable_lead_);

    Py_Initialize();
    if(!Py_IsInitialized()){
        std::cout << "python init failed" << std::endl;
        return false;
    }
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/apollo_workspace/cipv_fusion_component/component/python')");

    PyObject* module = PyImport_ImportModule("hello_world");
    // boost::python::object module(import("hello_world"));
    if(module == nullptr){
        std::cout << "module not found: hello_world" << std::endl;
        return false;
    }

    PyObject* func = PyObject_GetAttrString(module, "say");
    // module.attr("say");

    if(!func || !PyCallable_Check(func)){
        std::cout << "function not found: say" << std::endl;
        return false;
    }

    PyObject_CallObject(func, nullptr);

    Py_Finalize();
    // if (status) {
    //     fusion_writer_->Write(out_msg);
    //     AINFO << "Send cipv output message.";
    // }
    return true;
}
