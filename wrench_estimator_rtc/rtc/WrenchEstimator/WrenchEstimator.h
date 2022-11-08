#ifndef VirtualForceSensor_H
#define VirtualForceSensor_H

#include <memory>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/EigenUtil>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>

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
  std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq> > > m_wrenchesIn_;

public:
  WrenchEstimator(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

private:
  struct WrenchEstimatorParam {
    std::string target_name;
    cnoid::Vector3 p;
    cnoid::Matrix3 R;
    cnoid::Vector3 forceOffset = cnoid::Vector3(0,0,0);
    cnoid::Vector3 momentOffset = cnoid::Vector3(0,0,0);
  };
  std::map<std::string, std::shared_ptr<WrenchEstimatorParam>> m_sensors;
  cnoid::BodyPtr robot_;

  cnoid::VectorXd tau_act; // RobotHardwareからの生のトルク
  cnoid::VectorXd tau_g; // 重力補償分のトルク
  cnoid::VectorXd tau_ee; // 6軸力センサで計測されているレンチ分のトルク

};

extern "C"
{
  void WrenchEstimatorInit(RTC::Manager* manager);
}

#endif // VirtualForceSensor_H
