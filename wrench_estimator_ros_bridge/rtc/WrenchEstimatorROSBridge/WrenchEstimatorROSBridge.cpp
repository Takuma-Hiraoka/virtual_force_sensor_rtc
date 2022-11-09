#include "WrenchEstimatorROSBridge.h"

WrenchEstimatorROSBridge::WrenchEstimatorROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_estWrenchesIn_("estWrenchesIn", m_estWrenches_)
{
}

RTC::ReturnCode_t WrenchEstimatorROSBridge::onInitialize(){
  addInPort("estWrenchesIn", this->m_estWrenchesIn_);
  ros::NodeHandle pnh("~");

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
    //cnoid::Vector3 localp;
    //cnoid::Vector3 localaxis;
    //double localangle;

    name = buf;
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; target_name = buf;
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; //localp[0] = std::stod(buf);
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; //localp[1] = std::stod(buf);
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; //localp[2] = std::stod(buf);
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; //localaxis[0] = std::stod(buf);
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; //localaxis[1] = std::stod(buf);
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; //localaxis[2] = std::stod(buf);
    if(!std::getline(ss_wrenchEstimators, buf, ',')) break; //localangle = std::stod(buf);

    // check validity
    name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
    target_name.erase(std::remove(target_name.begin(), target_name.end(), ' '), target_name.end()); // remove whitespace

    this->wrench_pub[target_name] = nh.advertise<geometry_msgs::WrenchStamped>(name, 10);
  }
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t WrenchEstimatorROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();
  if(this->m_estWrenchesIn_.isNew()){
    this->m_estWrenchesIn_.read();
    for (int i = 0; i < this->m_estWrenches_.data.length(); i++){
      std::ostringstream link;
      link << this->m_estWrenches_.data[i].link;
      std::string link_name = link.str();
      if (wrench_pub.find(link_name) == wrench_pub.end()) { // 該当接触点が初期化時に登録されていない
	std::cerr << "[" << this->m_profile.instance_name << "] wrench_estimator doesn't have " << this->m_estWrenches_.data[i].link << std::endl;
      }else{
	// JOINT名をLINK名になおす
	std::string frame = link_name;
	unsigned int start_pos = frame.find("JOINT");
        frame.replace(start_pos, 5, "LINK");
	geometry_msgs::WrenchStamped estWrench;
	estWrench.header.stamp = ros::Time(this->m_estWrenches_.tm.sec, this->m_estWrenches_.tm.nsec);
	estWrench.header.frame_id = frame;
	estWrench.wrench.force.x = this->m_estWrenches_.data[i].force.x;
	estWrench.wrench.force.y = this->m_estWrenches_.data[i].force.y;
	estWrench.wrench.force.z = this->m_estWrenches_.data[i].force.z;
	estWrench.wrench.torque.x = this->m_estWrenches_.data[i].moment.x;
	estWrench.wrench.torque.y = this->m_estWrenches_.data[i].moment.y;
	estWrench.wrench.torque.z = this->m_estWrenches_.data[i].moment.z;
	wrench_pub[link_name].publish(estWrench);
      }
    }
  }

  return RTC::RTC_OK;
}


static const char* WrenchEstimatorROSBridge_spec[] = {
  "implementation_id", "WrenchEstimatorROSBridge",
  "type_name",         "WrenchEstimatorROSBridge",
  "description",       "WrenchEstimatorROSBridge component",
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
    void WrenchEstimatorROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(WrenchEstimatorROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<WrenchEstimatorROSBridge>, RTC::Delete<WrenchEstimatorROSBridge>);
    }
};
