#include "WrenchEstimatorROSBridge.h"

WrenchEstimatorROSBridge::WrenchEstimatorROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t WrenchEstimatorROSBridge::onInitialize(){

  ros::NodeHandle pnh("~");

  return RTC::RTC_OK;
}

RTC::ReturnCode_t WrenchEstimatorROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();
  return RTC::RTC_OK;
}


static const char* WrenchEstimatorROSBridge_spec[] = {
  "implementation_id", "WrenchEstimatorROSBridge",
  "type_name",         "WrenchEstimatorROSBridge",
  "description",       "WrenchEstimatorROSBridge component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void WrenchEstimatorROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(WrenchEstimatorROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<WrenchEstimatorROSBridge>, RTC::Delete<WrenchEstimatorROSBridge>);
    }
};
