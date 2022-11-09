#include "WrenchEstimatorService_impl.h"
#include "WrenchEstimator.h"

WrenchEstimatorService_impl::WrenchEstimatorService_impl()
{
}

WrenchEstimatorService_impl::~WrenchEstimatorService_impl()
{
}

void WrenchEstimatorService_impl::setComp(WrenchEstimator *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean WrenchEstimatorService_impl::removeWrenchEstimatorOffset(const CORBA::Double tm)
{
  return comp_->removeWrenchEstimatorOffset(tm);
};
