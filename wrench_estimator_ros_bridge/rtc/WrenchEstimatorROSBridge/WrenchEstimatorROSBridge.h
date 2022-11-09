#ifndef WrenchEstimatorROSBridge_H
#define AutoStabilizerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <wrench_estimator_msgs/idl/WrenchEstimator.hh>
#include "geometry_msgs/WrenchStamped.h"

#include <ros/ros.h>

class WrenchEstimatorROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;

  wrench_estimator_msgs::TimedWrenches m_estWrenches_;
  RTC::InPort<wrench_estimator_msgs::TimedWrenches> m_estWrenchesIn_;

  std::map<std::string, ros::Publisher> wrench_pub;
public:
  WrenchEstimatorROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

extern "C"
{
  void WrenchEstimatorROSBridgeInit(RTC::Manager* manager);
};

#endif // WrenchEstimatorROSBridge_H
