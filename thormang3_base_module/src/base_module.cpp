/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "thormang3_base_module/base_module.h"

#define EXT_PORT_DATA_1 "external_port_data_1"
#define EXT_PORT_DATA_2 "external_port_data_2"
#define EXT_PORT_DATA_3 "external_port_data_3"
#define EXT_PORT_DATA_4 "external_port_data_4"

using namespace thormang3;

BaseModule::BaseModule()
  : control_cycle_msec_(0),
    has_goal_joints_(false),
    ini_pose_only_(false)
{
  enable_       = false;
  module_name_  = "base_module";
  control_mode_ = robotis_framework::PositionControl;

  /* arm */
  result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]   = new robotis_framework::DynamixelState();

  /* gripper */
  result_["r_arm_grip"]   = new robotis_framework::DynamixelState();
  result_["l_arm_grip"]   = new robotis_framework::DynamixelState();

  /* body */
  result_["torso_y"]      = new robotis_framework::DynamixelState();

  /* leg */
  result_["r_leg_hip_y"]  = new robotis_framework::DynamixelState();
  result_["r_leg_hip_r"]  = new robotis_framework::DynamixelState();
  result_["r_leg_hip_p"]  = new robotis_framework::DynamixelState();
  result_["r_leg_kn_p"]   = new robotis_framework::DynamixelState();
  result_["r_leg_an_p"]   = new robotis_framework::DynamixelState();
  result_["r_leg_an_r"]   = new robotis_framework::DynamixelState();

  result_["l_leg_hip_y"]  = new robotis_framework::DynamixelState();
  result_["l_leg_hip_r"]  = new robotis_framework::DynamixelState();
  result_["l_leg_hip_p"]  = new robotis_framework::DynamixelState();
  result_["l_leg_kn_p"]   = new robotis_framework::DynamixelState();
  result_["l_leg_an_p"]   = new robotis_framework::DynamixelState();
  result_["l_leg_an_r"]   = new robotis_framework::DynamixelState();

  /* head */
  result_["head_y"]       = new robotis_framework::DynamixelState();
  result_["head_p"]       = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"]  = 1;
  joint_name_to_id_["l_arm_sh_p1"]  = 2;
  joint_name_to_id_["r_arm_sh_r"]   = 3;
  joint_name_to_id_["l_arm_sh_r"]   = 4;
  joint_name_to_id_["r_arm_sh_p2"]  = 5;
  joint_name_to_id_["l_arm_sh_p2"]  = 6;
  joint_name_to_id_["r_arm_el_y"]   = 7;
  joint_name_to_id_["l_arm_el_y"]   = 8;
  joint_name_to_id_["r_arm_wr_r"]   = 9;
  joint_name_to_id_["l_arm_wr_r"]   = 10;
  joint_name_to_id_["r_arm_wr_y"]   = 11;
  joint_name_to_id_["l_arm_wr_y"]   = 12;
  joint_name_to_id_["r_arm_wr_p"]   = 13;
  joint_name_to_id_["l_arm_wr_p"]   = 14;

  /* leg */
  joint_name_to_id_["r_leg_hip_y"]  = 15;
  joint_name_to_id_["l_leg_hip_y"]  = 16;
  joint_name_to_id_["r_leg_hip_r"]  = 17;
  joint_name_to_id_["l_leg_hip_r"]  = 18;
  joint_name_to_id_["r_leg_hip_p"]  = 19;
  joint_name_to_id_["l_leg_hip_p"]  = 20;
  joint_name_to_id_["r_leg_kn_p"]   = 21;
  joint_name_to_id_["l_leg_kn_p"]   = 22;
  joint_name_to_id_["r_leg_an_p"]   = 23;
  joint_name_to_id_["l_leg_an_p"]   = 24;
  joint_name_to_id_["r_leg_an_r"]   = 25;
  joint_name_to_id_["l_leg_an_r"]   = 26;

  /* body */
  joint_name_to_id_["torso_y"]      = 27;

  /* head */
  joint_name_to_id_["head_y"]       = 28;
  joint_name_to_id_["head_p"]       = 29;

  /* gripper */
  joint_name_to_id_["r_arm_grip"]   = 31;
  joint_name_to_id_["l_arm_grip"]   = 30;

  base_module_state_  = new BaseModuleState();
  joint_state_    = new BaseJointState();
}

BaseModule::~BaseModule()
{
  queue_thread_.join();
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_       = boost::thread(boost::bind(&BaseModule::queueThread, this));

  ros::NodeHandle ros_node;

  /* publish topics */
  status_msg_pub_       = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_  = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
}

void BaseModule::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  double mov_time;
  mov_time = doc["mov_time"].as<double>();

  base_module_state_->mov_time_ = mov_time;

  // parse via-point number
  int via_num;
  via_num = doc["via_num"].as<int>();

  base_module_state_->via_num_ = via_num;

  // parse via-point time
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  base_module_state_->via_time_.resize(via_num, 1);
  for (int num = 0; num < via_num; num++)
    base_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

  // parse via-point pose
  base_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
  base_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
  base_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

  base_module_state_->joint_via_pose_.fill(0.0);
  base_module_state_->joint_via_dpose_.fill(0.0);
  base_module_state_->joint_via_ddpose_.fill(0.0);

  YAML::Node via_pose_node = doc["via_pose"];
  for (YAML::iterator it = via_pose_node.begin(); it != via_pose_node.end(); ++it)
  {
    int id;
    std::vector<double> value;

    id = it->first.as<int>();
    value = it->second.as<std::vector<double> >();

    for (int num = 0; num < via_num; num++)
      base_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int id;
    double value;

    id = it->first.as<int>();
    value = it->second.as<double>();

    base_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;
  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

void BaseModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */

  // for gui
  ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/base/ini_pose", 5, &BaseModule::initPoseMsgCallback, this);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (base_module_state_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // set module of all joints -> this module
      setCtrlModule(module_name_);

      // wait to change module and to get goal position for init
      while (enable_ == false || has_goal_joints_ == false)
        usleep(8 * 1000);

      // parse initial pose
      std::string init_pose_path = ros::package::getPath("thormang3_base_module") + "/data/ini_pose.yaml";
      parseIniPoseData(init_pose_path);

      // generate trajectory
      tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::initPoseTrajGenerateProc, this));
    }
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void BaseModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (base_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  base_module_state_->smp_time_, base_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = base_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = base_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = base_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(base_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value,
                                                               0.0, 0.0, base_module_state_->smp_time_,
                                                               base_module_state_->via_time_, base_module_state_->mov_time_);
    }

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ROS_INFO("[start] send trajectory");
}

void BaseModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  setCtrlModule(module_name_);
  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  base_module_state_->mov_time_ = 5.0;
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  base_module_state_->joint_pose_ = joint_angle_pose;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_, base_module_state_->mov_time_);

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

void BaseModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
{
  setCtrlModule(module_name_);
  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  for (std::map<std::string, double>::iterator it = joint_angle_pose.begin(); it != joint_angle_pose.end(); it++)
  {
    std::string joint_name = it->first;
    double joint_angle_rad = it->second;

    std::map<std::string, int>::iterator joint_name_to_id_it = joint_name_to_id_.find(joint_name);
    if (joint_name_to_id_it != joint_name_to_id_.end())
    {
      target_pose.coeffRef(joint_name_to_id_it->second, 0) = joint_angle_rad;
    }
  }

  base_module_state_->joint_pose_ = target_pose;

  base_module_state_->mov_time_ = 5.0;
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_, base_module_state_->mov_time_);

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

bool BaseModule::isRunning()
{
  return base_module_state_->is_moving_;
}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
  }

  has_goal_joints_ = true;

  /* ----- send trajectory ----- */

  if (base_module_state_->is_moving_ == true)
  {
    if (base_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = base_module_state_->calc_joint_tra_(base_module_state_->cnt_, id);

    base_module_state_->cnt_++;
  }

  /*----- set joint data -----*/

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
  }

  /*---------- initialize count number ----------*/

  if ((base_module_state_->cnt_ >= base_module_state_->all_time_steps_) && (base_module_state_->is_moving_ == true))
  {
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

    base_module_state_->is_moving_ = false;
    base_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_ == true)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}

void BaseModule::stop()
{
  return;
}

void BaseModule::setCtrlModule(std::string module)
{
  std_msgs::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_.publish(control_msg);
}

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Base";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}
