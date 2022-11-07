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
#include <cnoid/EigenUtil>

class WrenchEstimator : public RTC::DataFlowComponentBase{
protected:

  RTC::TimedDoubleSeq m_q_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  RTC::TimedDoubleSeq m_dq_;
  RTC::InPort<RTC::TimedDoubleSeq> m_dqIn_;
  RTC::TimedDoubleSeq m_tau_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauIn_;
  RTC::TimedOrientation3D m_senRpy_;
  RTC::InPort<RTC::TimedOrientation3D> m_senRpyIn_;
  RTC::TimedAcceleration3D m_acc_;
  RTC::InPort<RTC::TimedAcceleration3D> m_accIn_;
  std::vector<RTC::TimedDoubleSeq> m_wrenches_;
  std::vector<RTC::InPort<RTC::TimedDoubleSeq>> m_wrenchesIn_;

public:
  WrenchEstimator(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

private:
  cnoid::BodyPtr robot_;

};

extern "C"
{
  void WrenchEstimatorInit(RTC::Manager* manager);
}

#endif // VirtualForceSensor_H
