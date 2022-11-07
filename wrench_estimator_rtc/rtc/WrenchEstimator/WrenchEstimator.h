#ifndef VirtualForceSensor_H
#define VirtualForceSensor_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <cnoid/EigenTypes>
#include <cnoid/Body>

class WrenchEstimator : public RTC::DataFlowComponentBase{
protected:

  //  RTC::TimedDoubleSeq m_q_;
  //  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  //  RTC::TimedOrientation3D m_baseRpy_;
  //  RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn_;

public:
  WrenchEstimator(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

private:
  //cnoid::BodyPtr robot_;

};

extern "C"
{
  void WrenchEstimatorInit(RTC::Manager* manager);
}

#endif // VirtualForceSensor_H
