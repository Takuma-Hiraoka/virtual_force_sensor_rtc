#include "WrenchEstimator.h"
#include <cnoid/BodyLoader>
#include <cnoid/RateGyroSensor>

WrenchEstimator::WrenchEstimator(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_qIn_("qIn", m_q_),
  m_dqIn_("dqIn", m_dq_),
  m_tauIn_("tauIn", m_tau_),
  m_senRpyIn_("senRpyIn", m_senRpy_),
  m_accIn_("accIn", m_acc_)
{
}

RTC::ReturnCode_t WrenchEstimator::onInitialize(){
  addInPort("qIn", this->m_qIn_);
  addInPort("dqIn", this->m_dqIn_);
  addInPort("tauIn", this->m_tauIn_);
  addInPort("senRpyIn", this->m_senRpyIn_);
  addInPort("accIn", this->m_accIn_);

  // load robot model
  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t WrenchEstimator::onExecute(RTC::UniqueId ec_id){
  std::cerr << "WrenchEstimator rtc onExecute" << std::endl;

  if (this->m_qIn_.isNew()){
    this->m_qIn_.read();
    if(this->m_q_.data.length() == this->robot_->numJoints()){
      for ( int i = 0; i < this->robot_->numJoints(); i++ ){
	this->robot_->joint(i)->q() = this->m_q_.data[i];
      }
    }
  }

  if(this->m_senRpyIn_.isNew()){
    this->m_senRpyIn_.read();
    this->robot_->calcForwardKinematics();
    cnoid::RateGyroSensorPtr imu = this->robot_->findDevice<cnoid::RateGyroSensor>("gyrometer");
    cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
    cnoid::Matrix3 actR = cnoid::rotFromRpy(m_senRpy_.data.r, m_senRpy_.data.p, m_senRpy_.data.y);
    this->robot_->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * this->robot_->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
  }

  if (this->m_dqIn_.isNew()){
    this->m_dqIn_.read();
    if(this->m_dq_.data.length() == this->robot_->numJoints()){
      for ( int i = 0; i < this->robot_->numJoints(); i++ ){
	this->robot_->joint(i)->dq() = this->m_dq_.data[i];
      }
    }
  }

  if (this->m_tauIn_.isNew()){
    this->m_tauIn_.read();
    if(this->m_tau_.data.length() == this->robot_->numJoints()){
      for ( int i = 0; i < this->robot_->numJoints(); i++ ){
	this->robot_->joint(i)->u() = this->m_tau_.data[i];
      }
    }
  }

  if (this->m_accIn_.isNew()) this->m_accIn_.read();

  this->robot_->calcForwardKinematics();
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
