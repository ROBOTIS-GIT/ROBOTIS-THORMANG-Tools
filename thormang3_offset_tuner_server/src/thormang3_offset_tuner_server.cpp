/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * thormang3_offset_tuner_server.cpp
 *
 *  Created on: 2016. 2. 15.
 *      Author: Jay Song
 */

#include "thormang3_offset_tuner_server/thormang3_offset_tuner_server.h"

#define OFFSET_ROSPARAM_KEY "offset"
#define OFFSET_INIT_POS_ROSPARAM_KEY "init_pose_for_offset_tuner"

using namespace thormang3;

OffsetTunerServer::OffsetTunerServer()
  : controller_(0),
    init_file_(""),
    robot_file_(""),
    offset_file_("")
{
}

OffsetTunerServer::~OffsetTunerServer()
{
}

void OffsetTunerServer::setCtrlModule(std::string module)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::map<std::string, robotis_framework::DynamixelState *>::iterator joint_iter;
  ros::NodeHandle nh;
  ros::Publisher set_ctrl_module_pub = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 1);

  BaseModule* base_module = BaseModule::getInstance();

  for (joint_iter = base_module->result_.begin(); joint_iter != base_module->result_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->first);
    control_msg.module_name.push_back(module);
  }

  set_ctrl_module_pub.publish(control_msg);
}

bool OffsetTunerServer::initialize()
{
  controller_ = robotis_framework::RobotisController::getInstance();

  ros::NodeHandle nh;

  //Get File Path
  nh.param<std::string>("offset_file_path", offset_file_, "");
  nh.param<std::string>("robot_file_path", robot_file_, "");
  nh.param<std::string>("init_file_path", init_file_, "");

  if ((offset_file_ == "") || (robot_file_ == ""))
  {
    ROS_ERROR("Failed to get file path");
    return -1;
  }
  //
  //Controller Initialize with robot file info
  if (controller_->initialize(robot_file_, init_file_) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }
  //controller_->LoadOffset(offset_file_);
  controller_->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());

  //Initialize RobotOffsetData
  for (std::map<std::string, robotis_framework::Dynamixel *>::iterator robot_it = controller_->robot_->dxls_.begin();
       robot_it != controller_->robot_->dxls_.end(); robot_it++)
  {
    std::string joint_name = robot_it->first;
    robot_offset_data_[joint_name] = new JointOffsetData(0, 0);
    robot_torque_enable_data_[joint_name] = true;
  }

  //Get and Initialize Offset Data
  std::map<std::string, double> _offset_data;
  //bool result = nh.getParam(OFFSET_ROSPARAM_KEY, _offset_data);
//	if(result == true)
//	{
  //ROS_INFO("Succeed to get offset");

  //offset data initialize
  //referring to  robotis file info the data will be initialized

//
//
//
//		for(std::map<std::string, double>::iterator _it = _offset_data.begin(); _it != _offset_data.end(); _it++)
//	    {
//	        std::string _joint_name = _it->first;
//	        double 		_joint_offset = _it->second;
//
//	        RobotOffsetData[_joint_name]		= new JointOffsetData(_joint_offset, 0);
//	        RobotTorqueEnableData[_joint_name]	= true;
//	    }
//	}
//	else {
//		ROS_ERROR("Failed to get offset data");
//		return false;
//	}

  //Load Offset.yaml
  YAML::Node offset_yaml_node = YAML::LoadFile(offset_file_.c_str());

  //Get Offset Data and Init_Pose for Offset Tuning
  YAML::Node offset_data_node = offset_yaml_node[OFFSET_ROSPARAM_KEY];
  YAML::Node offset_init_pose_node = offset_yaml_node[OFFSET_INIT_POS_ROSPARAM_KEY];

  //Initialize Offset Data in RobotOffsetData
  for (YAML::const_iterator node_it = offset_data_node.begin(); node_it != offset_data_node.end(); node_it++)
  {
    std::string joint_name = node_it->first.as<std::string>();
    double offset = node_it->second.as<double>();

    std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data_.find(joint_name);
    if (robot_offset_data_it != robot_offset_data_.end())
      robot_offset_data_[joint_name]->joint_offset_rad_ = offset;
  }

  //Initialize Init Pose for Offset Tuning in RobotOffsetData
  for (YAML::const_iterator ode_it = offset_init_pose_node.begin(); ode_it != offset_init_pose_node.end(); ode_it++)
  {
    std::string joint_name = ode_it->first.as<std::string>();
    double offset_init_pose = ode_it->second.as<double>();

    std::map<std::string, JointOffsetData*>::iterator robot_offset_data_it = robot_offset_data_.find(joint_name);
    if (robot_offset_data_it != robot_offset_data_.end())
      robot_offset_data_[joint_name]->joint_init_pos_rad_ = offset_init_pose;
  }

//	//Get Initial Pose for Offset Tunning
//	std::map<std::string, double> _init;
//	result = _nh.getParam(OFFSET_INIT_POS_ROSPARAM_KEY, _init);
//	if(result == true)
//	{
//		std::map<std::string, JointOffsetData*>::iterator _robot_offset_data_it; // iterator for RobotOffsetData
//		ROS_INFO("Succeed to get offset init pose");
//	    for(std::map<std::string, double>::iterator _it = _init.begin(); _it != _init.end(); _it++) {
//	        std::string _joint_name = _it->first;
//	        double 		_joint_init = _it->second;
//
//	        //check the joint name exist at the offset data
//	        _robot_offset_data_it = RobotOffsetData.find(_joint_name);
//
//	        if(_robot_offset_data_it != RobotOffsetData.end())
//	        	RobotOffsetData[_joint_name]->joint_init_pos_rad = _joint_init;
//	        else
//	        	continue;
//	    }
//	}
//	else {
//		ROS_ERROR("Failed to get offset init pose");
//		return false;
//	}

  ROS_INFO(" ");
  ROS_INFO(" ");
  //For Data Check, Print All Available Data
  int i = 0;
  ROS_INFO_STREAM("num" << " | " << "joint_name" << " | " << "InitPose" << ", " << "OffsetAngleRad");
  for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin(); it != robot_offset_data_.end(); it++)
  {
    std::string       joint_name = it->first;
    JointOffsetData  *joint_data = it->second;

    ROS_INFO_STREAM(i << " | " << joint_name << " : " << joint_data->joint_init_pos_rad_ << ", "
                    << joint_data->joint_offset_rad_);
    i++;
  }

  send_tra_sub_             = nh.subscribe("/robotis/base/send_tra", 10, &OffsetTunerServer::stringMsgsCallBack, this);
  joint_offset_data_sub_    = nh.subscribe("/robotis/offset_tuner/joint_offset_data", 10,
                                           &OffsetTunerServer::jointOffsetDataCallback, this);
  joint_torque_enable_sub_  = nh.subscribe("/robotis/offset_tuner/torque_enable", 10,
                                           &OffsetTunerServer::jointTorqueOnOffCallback, this);
  command_sub_              = nh.subscribe("/robotis/offset_tuner/command", 5, &OffsetTunerServer::commandCallback, this);
  offset_data_server_       = nh.advertiseService("robotis/offset_tuner/get_present_joint_offset_data",
                                                  &OffsetTunerServer::getPresentJointOffsetDataServiceCallback, this);

  return true;
}

void OffsetTunerServer::moveToInitPose()
{

  controller_->startTimer();
  BaseModule *base_module = BaseModule::getInstance();

//	Eigen::MatrixXd _offset_init_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

//    for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//    {
//        std::string 		_joint_name = _it->first;
//        JointOffsetData* 	_joint_data = _it->second;
//
//
//        if ( controller_->robot->dxls.find(_joint_name) == controller_->robot->dxls.end() ) {
//          continue;
//        }
//        else {
//            int id = (int)(controller_->robot->dxls[_joint_name]->id);
//            //if(id > MAX_JOINT_ID) {
//            if(id > MAX_JOINT_ID) {
//            	ROS_ERROR("Invalid JointID");
//            	return;
//            }
//            else {
//            	offset_init_pose.coeffRef(id, 0) = _joint_data->joint_init_pos_rad;
//            }
//        }
//    }

  //make map, key : joint_name, value : joint_init_pos_rad;
  std::map<std::string, double> offset_init_pose;
  for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin();
       it != robot_offset_data_.end(); it++)
  {
    std::string       joint_name = it->first;
    JointOffsetData  *joint_data = it->second;

    if (controller_->robot_->dxls_.find(joint_name) == controller_->robot_->dxls_.end())
      continue;
    else
      offset_init_pose[joint_name] = joint_data->joint_init_pos_rad_ + joint_data->joint_offset_rad_;
  }

  usleep(80 * 1000);
  base_module->poseGenerateProc(offset_init_pose);
  usleep(10 * 000);

  while (base_module->isRunning())
    usleep(50 * 1000);

  controller_->stopTimer();
  while (controller_->isTimerRunning())
    usleep(10 * 1000);

  if (controller_->isTimerRunning())
  {
    ROS_INFO("Timer Running");
  }

  setCtrlModule("none");
}

void OffsetTunerServer::stringMsgsCallBack(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->data);
}

void OffsetTunerServer::commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "save")
  {
//		YAML::Node  _baseNode = YAML::LoadFile(offset_file_); // gets the root node
//	    for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//	    {
//	        std::string 		_joint_name = _it->first;
//	        JointOffsetData* 	_joint_data = _it->second;
//
//	        _baseNode["offset"][_joint_name] 					 = _joint_data->joint_offset_rad; // edit one of the nodes
//	        _baseNode["init_pose_for_offset_tuner"][_joint_name] = _joint_data->joint_init_pos_rad; // edit one of the nodes
//	    }
//		std::ofstream fout(offset_file_.c_str());
//		fout << _baseNode; // dump it back into the file

    YAML::Emitter out;
    std::map<std::string, double> offset;
    std::map<std::string, double> init_pose;
    for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin();
         it != robot_offset_data_.end(); it++)
    {
      std::string       joint_name = it->first;
      JointOffsetData  *joint_data = it->second;

      offset[joint_name]    = joint_data->joint_offset_rad_; // edit one of the nodes
      init_pose[joint_name] = joint_data->joint_init_pos_rad_; // edit one of the nodes
    }

    out << YAML::BeginMap;
    out << YAML::Key << "offset" << YAML::Value << offset;
    out << YAML::Key << "init_pose_for_offset_tuner" << YAML::Value << init_pose;
    out << YAML::EndMap;
    std::ofstream fout(offset_file_.c_str());
    fout << out.c_str(); // dump it back into the file

  }
  else if (msg->data == "ini_pose")
    moveToInitPose();
  else
    ROS_INFO_STREAM("Invalid Command : " << msg->data);
}

void OffsetTunerServer::jointOffsetDataCallback(const thormang3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg)
{
  if (controller_->isTimerRunning())
  {
    ROS_ERROR("Timer is running now");
    return;
  }

  //goal position
  ROS_INFO_STREAM(msg->joint_name << " " << msg->goal_value << " " << msg->offset_value
                  << " " << msg->p_gain << " " << msg->i_gain << " " << msg->d_gain);

  std::map<std::string, JointOffsetData*>::iterator it;
  it = robot_offset_data_.find(msg->joint_name);
  if (it == robot_offset_data_.end())
  {
    ROS_ERROR("Invalid Joint Name");
    return;
  }

  if (robot_torque_enable_data_[msg->joint_name] == false)
  {
    ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
    return;
  }

  double  goal_pose_rad   = msg->offset_value + msg->goal_value;
  int32_t goal_pose_value = controller_->robot_->dxls_[msg->joint_name]->convertRadian2Value(goal_pose_rad);
  uint8_t dxl_error       = 0;
  int32_t comm_result     = COMM_SUCCESS;

  comm_result = controller_->writeCtrlItem(msg->joint_name,
                                           controller_->robot_->dxls_[msg->joint_name]->goal_position_item_->item_name_,
                                           goal_pose_value, &dxl_error);
  if (comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to write goal position");
    return;
  }
  else
  {
    robot_offset_data_[msg->joint_name]->joint_init_pos_rad_  = msg->goal_value;
    robot_offset_data_[msg->joint_name]->joint_offset_rad_    = msg->offset_value;
  }

  if (dxl_error != 0)
  {
    ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error " << (int) dxl_error);
  }

  robot_offset_data_[msg->joint_name]->p_gain_ = msg->p_gain;
  robot_offset_data_[msg->joint_name]->i_gain_ = msg->i_gain;
  robot_offset_data_[msg->joint_name]->d_gain_ = msg->d_gain;

}

void OffsetTunerServer::jointTorqueOnOffCallback(
    const thormang3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg)
{
  for (unsigned int i = 0; i < msg->torque_enable_data.size(); i++)
  {
    std::string joint_name = msg->torque_enable_data[i].joint_name;
    bool torque_enable = msg->torque_enable_data[i].torque_enable;
    ROS_INFO_STREAM(i << " " << joint_name << torque_enable);

    std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.find(joint_name);
    if (it == robot_offset_data_.end())
    {
      ROS_ERROR("Invalid Joint Name");
      continue;
    }
    else
    {
      int32_t comm_result = COMM_SUCCESS;
      uint8_t dxl_error = 0;
      uint8_t torque_enable_value = 0;

      if (torque_enable)
        torque_enable_value = 1;
      else
        torque_enable_value = 0;

      comm_result = controller_->writeCtrlItem(joint_name,
                                               controller_->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
                                               torque_enable_value, &dxl_error);
      if (comm_result != COMM_SUCCESS)
      {
        ROS_ERROR("Failed to write goal position");
      }
      else
      {
        robot_torque_enable_data_[joint_name] = torque_enable;
      }

      if (dxl_error != 0)
      {
        ROS_ERROR_STREAM("goal_pos_set : " << joint_name << "  has error " << (int) dxl_error);
      }
    }
  }
}

bool OffsetTunerServer::getPresentJointOffsetDataServiceCallback(
                        thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
                        thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res)
{

  ROS_INFO("GetPresentJointOffsetDataService Called");

  for (std::map<std::string, JointOffsetData*>::iterator it = robot_offset_data_.begin();
       it != robot_offset_data_.end(); it++)
  {
    std::string       joint_name = it->first;
    JointOffsetData  *joint_data = it->second;

    thormang3_offset_tuner_msgs::JointOffsetPositionData joint_offset_pos;

    int32_t present_pos_value = 0;
    uint8_t dxl_error         = 0;
    int     comm_result       = COMM_SUCCESS;

    comm_result = controller_->readCtrlItem(joint_name,
                                            controller_->robot_->dxls_[joint_name]->present_position_item_->item_name_,
                                            (uint32_t*) &present_pos_value,
                                            &dxl_error);
    if (comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("Failed to read present pos");
      return false;
    }
    else
    {
      if (dxl_error != 0)
      {
        ROS_ERROR_STREAM(joint_name << "  has error " << (int) dxl_error);
      }

      joint_offset_pos.joint_name     = joint_name;
      joint_offset_pos.goal_value     = joint_data->joint_init_pos_rad_;
      joint_offset_pos.offset_value   = joint_data->joint_offset_rad_;
      joint_offset_pos.present_value  = controller_->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value);
      joint_offset_pos.p_gain         = joint_data->p_gain_;
      joint_offset_pos.i_gain         = joint_data->i_gain_;
      joint_offset_pos.d_gain         = joint_data->d_gain_;

      res.present_data_array.push_back(joint_offset_pos);
    }
  }
  return true;
}

