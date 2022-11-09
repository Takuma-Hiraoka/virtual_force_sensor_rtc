#ifndef WrenchEstimatorSERVICESVC_IMPL_H
#define WrenchEstimatorSERVICESVC_IMPL_H

#include "wrench_estimator_rtc/idl/WrenchEstimatorService.hh"

class WrenchEstimator;

class WrenchEstimatorService_impl
  : public virtual POA_wrench_estimator_rtc::WrenchEstimatorService,
    public virtual PortableServer::RefCountServantBase
{
public:
  WrenchEstimatorService_impl();
  ~WrenchEstimatorService_impl();

  CORBA::Boolean removeWrenchEstimatorOffset(const CORBA::Double tm);
  void setComp(WrenchEstimator *i_comp);
private:
  WrenchEstimator *comp_;
};

#endif
