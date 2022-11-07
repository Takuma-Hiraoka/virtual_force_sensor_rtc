#include "WrenchEstimator.h"

WrenchEstimator::WrenchEstimator(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
  //  m_qIn_("qIn", m_q_),
  //  m_baseRpyIn_("baseRpyIn", m_baseRpy_)
{
}

RTC::ReturnCode_t WrenchEstimator::onInitialize(){
  //addInPort("qIn", this->m_qIn_);
  //addInPort("baseRpyIn", this->m_baseRpyIn_);
  std::cerr << "WrenchEstimator rtc onInitialize" << std::endl;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t WrenchEstimator::onExecute(RTC::UniqueId ec_id){
  std::cerr << "WrenchEstimator rtc onExecute" << std::endl;

  return RTC::RTC_OK;
}

static const char* WrenchEstimator_spec[] = {
  "implementation_id", "WrenchEstimator",
  "type_name",         "WrenchEstimator",
  "description",       "WrenchEstimator component",
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
  void WrenchEstimatorInit(RTC::Manager* manager) {
    RTC::Properties profile(WrenchEstimator_spec);
    manager->registerFactory(profile, RTC::Create<WrenchEstimator>, RTC::Delete<WrenchEstimator>);
  }
};
