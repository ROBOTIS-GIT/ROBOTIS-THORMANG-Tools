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

#ifndef THORMANG3_OFFSET_TUNER_SERVER_H_
#define THORMANG3_OFFSET_TUNER_SERVER_H_

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "robotis_controller/robotis_controller.h"
#include "thormang3_base_module/base_module.h"
#include "thormang3_offset_tuner_msgs/JointOffsetData.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "thormang3_offset_tuner_msgs/GetPresentJointOffsetData.h"

namespace thormang3
{

class JointOffsetData
{
public:
  double joint_offset_rad_;
  double joint_init_pos_rad_;
  int p_gain_;
  int i_gain_;
  int d_gain_;

  JointOffsetData()
  {
    joint_offset_rad_ = 0;
    joint_init_pos_rad_ = 0;
    p_gain_ = 32;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  JointOffsetData(double joint_offset_rad, double joint_init_pose_rad)
  {
    this->joint_offset_rad_ = joint_offset_rad;
    this->joint_init_pos_rad_ = joint_init_pose_rad;
    p_gain_ = 32;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  ~JointOffsetData()
  {
  }
};

class OffsetTunerServer: public robotis_framework::Singleton<OffsetTunerServer>
{
private:
  //RobotisController* controller_;

  robotis_framework::RobotisController* controller_;

  std::string init_file_;
  std::string robot_file_;
  std::string offset_file_;

  std::map<std::string, bool>             robot_torque_enable_data_;
  std::map<std::string, JointOffsetData*> robot_offset_data_;

  ros::Subscriber     send_tra_sub_;
  ros::Subscriber     joint_offset_data_sub_;
  ros::Subscriber     joint_torque_enable_sub_;
  ros::Subscriber     command_sub_;
  ros::ServiceServer  offset_data_server_;

  void setCtrlModule(std::string module);

public:
  OffsetTunerServer();
  ~OffsetTunerServer();

  bool initialize();
  void moveToInitPose();
  void stringMsgsCallBack(const std_msgs::String::ConstPtr& msg);
  void commandCallback(const std_msgs::String::ConstPtr& msg);
  void jointOffsetDataCallback(const thormang3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg);
  void jointTorqueOnOffCallback(const thormang3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg);
  bool getPresentJointOffsetDataServiceCallback(thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
                                                thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res);

};

}

#endif /* THORMANG3_OFFSET_TUNER_SERVER_H_ */
