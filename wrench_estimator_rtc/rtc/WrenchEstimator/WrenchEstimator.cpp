#include "WrenchEstimator.h"
#include <cnoid/BodyLoader>
#include <cnoid/RateGyroSensor>
#include <cnoid/ForceSensor>

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

  // 各ForceSensorにつき、<name>InというInportをつくる
  cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->robot_->devices());
  this->m_wrenchesIn_.resize(forceSensors.size());
  this->m_wrenches_.resize(forceSensors.size());
  for(int i=0;i<forceSensors.size();i++){
    std::string name = forceSensors[i]->name()+"In";
    this->m_wrenchesIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->m_wrenches_[i]);
    this->addInPort(name.c_str(), *(m_wrenchesIn_[i]));
  }

  {
    // wrench_estimator: <name>, <target>, 0, 0, 0,  0, 0, 1, 0
    std::string wrenchEstimators;
    if(this->getProperties().hasKey("wrench_estimator")) wrenchEstimators = std::string(this->getProperties()["wrench_estimator"]);
    else wrenchEstimators = std::string(this->m_pManager->getConfig()["wrench_estimator"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] wrench_estimator : " << wrenchEstimators << std::endl;
    std::stringstream ss_wrenchEstimators(wrenchEstimators);
    std::string buf;
    while(std::getline(ss_wrenchEstimators, buf, ',')){
      std::string name;
      std::string target_name;
      cnoid::Vector3 localp;
      cnoid::Vector3 localaxis;
      double localangle;

      name = buf;
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; target_name = buf;
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_wrenchEstimators, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      target_name.erase(std::remove(target_name.begin(), target_name.end(), ' '), target_name.end()); // remove whitespace

      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();

      std::cerr << "[" << this->m_profile.instance_name << "] wrench estimator : " << name << std::endl;
      std::cerr << "[" << this->m_profile.instance_name << "]           target : " << target_name << std::endl;
      std::cerr << "[" << this->m_profile.instance_name << "]             T, R : " << localp[0] << " " << localp[1] << " " << localp[2] << std::endl << localR << std::endl;
      WrenchEstimator::WrenchEstimatorParam wep;
      wep.target_name = target_name;
      wep.p = localp;
      wep.R = localR;
      wep.path = cnoid::JointPath(this->robot_->rootLink(), this->robot_->link(target_name));
      m_sensors[name] = wep;
    }
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
