#include "WrenchEstimator.h"
#include <cnoid/BodyLoader>
#include <cnoid/RateGyroSensor>
#include <cnoid/ForceSensor>
#include <cnoid/src/Body/InverseDynamics.h>

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

  this->tau_act = cnoid::VectorXd::Zero(6 + this->robot_->numJoints());
  this->tau_g = cnoid::VectorXd::Zero(6 + this->robot_->numJoints());
  this->tau_ee = cnoid::VectorXd::Zero(6 + this->robot_->numJoints());

  return RTC::RTC_OK;
}

RTC::ReturnCode_t WrenchEstimator::onExecute(RTC::UniqueId ec_id){
  std::cerr << "WrenchEstimator rtc onExecute" << std::endl;

  cnoid::VectorXd Tvirtual = cnoid::VectorXd::Zero(6 + this->robot_->numJoints());

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
	tau_act[6+i] = this->m_tau_.data[i];
      }
    }
    Tvirtual -= this->tau_act;
  }

  if (this->m_accIn_.isNew()) this->m_accIn_.read();
  for (size_t i = 0; i < m_wrenchesIn_.size(); ++i) {
      if ( m_wrenchesIn_[i]->isNew() ) {
          m_wrenchesIn_[i]->read();
      }
  }

  this->robot_->calcForwardKinematics();

  cnoid::Vector3 base_f = cnoid::Vector3::Zero();
  cnoid::Vector3 base_t = cnoid::Vector3::Zero();
  
  cnoid::calcInverseDynamics(this->robot_->rootLink()); // 逆動力学を解く 重力補償分のtau_gが求まる
  this->tau_g.block<3,1>(0,0) = base_f;
  this->tau_g.block<3,1>(3,0) = base_t;
  for (int i=0; i< this->robot_->numJoints(); i++){
    this->tau_g[6+i] = this->robot_->joint(i)->u();
  }

  Tvirtual += this->tau_g;

  //各反力に相当するトルク
  cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->robot_->devices());
  for (size_t i = 0; i < forceSensors.size() ; i++){
    cnoid::JointPath jointpath(this->robot_->rootLink(), this->robot_->devices()[i]->link());
    cnoid::MatrixXd JJ = cnoid::MatrixXd::Zero(6,jointpath.numJoints());
    cnoid::setJacobian<0x3f,0,0>(jointpath,this->robot_->devices()[i]->link(), // input
				      JJ); // output 
    cnoid::MatrixXd J = cnoid::MatrixXd::Zero(6,6+this->robot_->numJoints());
    J.block<3,3>(0,0) = cnoid::Matrix3::Identity();
    J.block<3,3>(0,3) = - cnoid::hat(forceSensors[i]->link()->p() + forceSensors[i]->link()->R() * forceSensors[i]->p_local());
    J.block<3,3>(3,3) = cnoid::Matrix3::Identity();
    for (int j = 0; j < jointpath.numJoints() ; j++){
      J.block<6,1>(0,6+jointpath.joint(j)->jointId()) = JJ.block<6,1>(0,j);
    }
    
    //実際の反力を取得
    cnoid::Vector6 wrench = cnoid::Vector6::Zero();
    wrench << m_wrenches_[i].data[0],m_wrenches_[i].data[1],m_wrenches_[i].data[2],m_wrenches_[i].data[3],m_wrenches_[i].data[4],m_wrenches_[i].data[5];//実際の反力を取得
    wrench.block<3,1>(0,0) = (forceSensors[i]->link()->R() * forceSensors[i]->R_local()) * wrench.head<3>();
    wrench.block<3,1>(3,0) = (forceSensors[i]->link()->R() * forceSensors[i]->R_local()) * wrench.tail<3>();

    Tvirtual -= J.transpose() * wrench;//このセンサのJ^T wを引く
    
    }
  //トルクを引く Tvirtual = J^T w^e になっている

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
