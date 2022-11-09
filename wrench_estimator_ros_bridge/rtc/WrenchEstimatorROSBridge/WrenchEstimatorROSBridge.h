#ifndef WrenchEstimatorROSBridge_H
#define AutoStabilizerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <wrench_estimator_msgs/idl/WrenchEstimator.hh>

#include <ros/ros.h>

class WrenchEstimatorROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;
  
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
