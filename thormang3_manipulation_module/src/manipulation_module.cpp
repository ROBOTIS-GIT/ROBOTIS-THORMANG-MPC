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
 *  manipulation_module.cpp
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#include "thormang3_manipulation_module/manipulation_module.h"

using namespace thormang3;

ManipulationModule::ManipulationModule()
  : control_cycle_msec_(0)
{
  enable_       = false;
  module_name_  = "manipulation_module";
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
  result_["torso_y"]      = new robotis_framework::DynamixelState();
  result_["r_arm_grip"]   = new robotis_framework::DynamixelState();
  result_["l_arm_grip"]   = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id["r_arm_sh_p1"] = 1;
  joint_name_to_id["l_arm_sh_p1"] = 2;
  joint_name_to_id["r_arm_sh_r"]  = 3;
  joint_name_to_id["l_arm_sh_r"]  = 4;
  joint_name_to_id["r_arm_sh_p2"] = 5;
  joint_name_to_id["l_arm_sh_p2"] = 6;
  joint_name_to_id["r_arm_el_y"]  = 7;
  joint_name_to_id["l_arm_el_y"]  = 8;
  joint_name_to_id["r_arm_wr_r"]  = 9;
  joint_name_to_id["l_arm_wr_r"]  = 10;
  joint_name_to_id["r_arm_wr_y"]  = 11;
  joint_name_to_id["l_arm_wr_y"]  = 12;
  joint_name_to_id["r_arm_wr_p"]  = 13;
  joint_name_to_id["l_arm_wr_p"]  = 14;
  joint_name_to_id["torso_y"]     = 27;
  joint_name_to_id["r_arm_grip"]  = 31;
  joint_name_to_id["l_arm_grip"]  = 30;

  /* etc */
  joint_name_to_id["r_arm_end"]   = 35;
  joint_name_to_id["l_arm_end"]   = 34;

  /* parameter */
  humanoid_                   = new KinematicsDynamics(WholeBody);
  joint_state_                = new ManipulationJointState();
  manipulation_module_state_  = new ManipulationModuleState();
}

ManipulationModule::~ManipulationModule()
{
  queue_thread_.join();
}

void ManipulationModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_       = boost::thread(boost::bind(&ManipulationModule::queueThread, this));

  ros::NodeHandle ros_node;

  /* publish topics */

  // for gui
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);

  std::string _path = ros::package::getPath("thormang3_manipulation_module") + "/config/ik_weight.yaml";
  parseData(_path);
}

void ManipulationModule::parseData(const std::string &path)
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

  YAML::Node ik_weight_node = doc["weight_value"];
  for (YAML::iterator it = ik_weight_node.begin(); it != ik_weight_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    manipulation_module_state_->ik_weight_.coeffRef(id, 0) = value;
  }
}

void ManipulationModule::parseIniPoseData(const std::string &path)
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

  manipulation_module_state_->mov_time_ = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    manipulation_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  manipulation_module_state_->all_time_steps_ = int(manipulation_module_state_->mov_time_ / manipulation_module_state_->smp_time_) + 1;
  manipulation_module_state_->calc_joint_tra_.resize(manipulation_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

void ManipulationModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber ini_pose_msg_sub        = ros_node.subscribe("/robotis/manipulation/ini_pose_msg", 5,
                                                               &ManipulationModule::initPoseMsgCallback, this);
  ros::Subscriber joint_pose_msg_sub      = ros_node.subscribe("/robotis/manipulation/joint_pose_msg", 5,
                                                               &ManipulationModule::jointPoseMsgCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/manipulation/kinematics_pose_msg", 5,
                                                               &ManipulationModule::kinematicsPoseMsgCallback, this);

  /* service */
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/manipulation/get_joint_pose",
                                                                       &ManipulationModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/manipulation/get_kinematics_pose",
                                                                            &ManipulationModule::getKinematicsPoseCallback, this);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void ManipulationModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (manipulation_module_state_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // parse initial pose
      std::string ini_pose_path = ros::package::getPath("thormang3_manipulation_module") + "/config/ini_pose.yaml";
      parseIniPoseData(ini_pose_path);

      traj_generate_tread_ = new boost::thread(boost::bind(&ManipulationModule::initPoseTrajGenerateProc, this));
      delete traj_generate_tread_;
    }
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  return;
}

bool ManipulationModule::getJointPoseCallback(thormang3_manipulation_module_msgs::GetJointPose::Request &req,
                                              thormang3_manipulation_module_msgs::GetJointPose::Response &res)
{
  if (enable_ == false)
    return false;

  for (int name_index = 1; name_index <= MAX_JOINT_ID; name_index++)
  {
    if (humanoid_->thormang3_link_data_[name_index]->name_ == req.joint_name)
    {
      res.joint_value = joint_state_->goal_joint_state[name_index].position_;
      return true;
    }
  }

  return false;
}

bool ManipulationModule::getKinematicsPoseCallback(thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req,
                                                   thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res)
{
  if (enable_ == false)
    return false;

  int end_index;

  if (req.group_name == "left_arm")
    end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm")
    end_index = ID_R_ARM_END;
  else if (req.group_name == "left_arm_with_torso")
    end_index = ID_L_ARM_END;
  else if (req.group_name == "right_arm_with_torso")
    end_index = ID_R_ARM_END;
  else
    return false;

  res.group_pose.position.x = humanoid_->thormang3_link_data_[end_index]->position_.coeff(0, 0);
  res.group_pose.position.y = humanoid_->thormang3_link_data_[end_index]->position_.coeff(1, 0);
  res.group_pose.position.z = humanoid_->thormang3_link_data_[end_index]->position_.coeff(2, 0);

  Eigen::Quaterniond quaternion = robotis_framework::convertRotationToQuaternion(humanoid_->thormang3_link_data_[end_index]->orientation_);

  res.group_pose.orientation.w = quaternion.w();
  res.group_pose.orientation.x = quaternion.x();
  res.group_pose.orientation.y = quaternion.y();
  res.group_pose.orientation.z = quaternion.z();

  return true;
}

void ManipulationModule::kinematicsPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  manipulation_module_state_->goal_kinematics_pose_msg_ = *msg;

  if (manipulation_module_state_->goal_kinematics_pose_msg_.name == "left_arm")
  {
    manipulation_module_state_->ik_id_start_  = ID_L_ARM_START;
    manipulation_module_state_->ik_id_end_    = ID_L_ARM_END;
  }
  else if (manipulation_module_state_->goal_kinematics_pose_msg_.name == "right_arm")
  {
    manipulation_module_state_->ik_id_start_  = ID_R_ARM_START;
    manipulation_module_state_->ik_id_end_    = ID_R_ARM_END;
  }
  else if (manipulation_module_state_->goal_kinematics_pose_msg_.name == "left_arm_with_torso")
  {
    manipulation_module_state_->ik_id_start_  = ID_TORSO;
    manipulation_module_state_->ik_id_end_    = ID_L_ARM_END;
  }
  else if (manipulation_module_state_->goal_kinematics_pose_msg_.name == "right_arm_with_torso")
  {
    manipulation_module_state_->ik_id_start_  = ID_TORSO;
    manipulation_module_state_->ik_id_end_    = ID_R_ARM_END;
  }

  if (manipulation_module_state_->is_moving_ == false)
  {
    traj_generate_tread_ = new boost::thread(boost::bind(&ManipulationModule::taskTrajGenerateProc, this));
    delete traj_generate_tread_;
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  return;
}

void ManipulationModule::jointPoseMsgCallback(const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  manipulation_module_state_->goal_joint_pose_msg_ = *msg;

  if (manipulation_module_state_->is_moving_ == false)
  {
    traj_generate_tread_ = new boost::thread(boost::bind(&ManipulationModule::jointTrajGenerateProc, this));
    delete traj_generate_tread_;
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  return;
}

void ManipulationModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state[id].position_;
    double tar_value = manipulation_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                manipulation_module_state_->smp_time_,
                                                                manipulation_module_state_->mov_time_);

    manipulation_module_state_->calc_joint_tra_.block(0, id, manipulation_module_state_->all_time_steps_, 1) = tra;
  }

  manipulation_module_state_->cnt_        = 0;
  manipulation_module_state_->is_moving_  = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::jointTrajGenerateProc()
{
  if (manipulation_module_state_->goal_joint_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol        = 10 * DEGREE2RADIAN; // rad per sec
    double mov_time   = 2.0;

    int    ctrl_id    = joint_name_to_id[manipulation_module_state_->goal_joint_pose_msg_.name];

    double ini_value  = joint_state_->goal_joint_state[ctrl_id].position_;
    double tar_value  = manipulation_module_state_->goal_joint_pose_msg_.value;
    double diff       = fabs(tar_value - ini_value);

    manipulation_module_state_->mov_time_ = diff / tol;
    int _all_time_steps = int(floor((manipulation_module_state_->mov_time_ / manipulation_module_state_->smp_time_) + 1.0));
    manipulation_module_state_->mov_time_ = double(_all_time_steps - 1) * manipulation_module_state_->smp_time_;

    if (manipulation_module_state_->mov_time_ < mov_time)
      manipulation_module_state_->mov_time_ = mov_time;
  }
  else
  {
    manipulation_module_state_->mov_time_ = manipulation_module_state_->goal_joint_pose_msg_.time;
  }

  manipulation_module_state_->all_time_steps_ = int(manipulation_module_state_->mov_time_ / manipulation_module_state_->smp_time_) + 1;

  manipulation_module_state_->calc_joint_tra_.resize(manipulation_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  /* calculate joint trajectory */
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state[id].position_;
    double tar_value = joint_state_->goal_joint_state[id].position_;

    if (humanoid_->thormang3_link_data_[id]->name_ == manipulation_module_state_->goal_joint_pose_msg_.name)
      tar_value = manipulation_module_state_->goal_joint_pose_msg_.value;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                manipulation_module_state_->smp_time_,
                                                                manipulation_module_state_->mov_time_);

    manipulation_module_state_->calc_joint_tra_.block(0, id, manipulation_module_state_->all_time_steps_, 1) = tra;
  }

  manipulation_module_state_->cnt_        = 0;
  manipulation_module_state_->is_moving_  = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::taskTrajGenerateProc()
{
  if (manipulation_module_state_->goal_joint_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol      = 0.1; // m per sec
    double mov_time = 2.0;

    double diff     = sqrt(
                          pow(humanoid_->thormang3_link_data_[manipulation_module_state_->ik_id_end_]->position_.coeff(0, 0)
                              - manipulation_module_state_->goal_kinematics_pose_msg_.pose.position.x, 2)
                        + pow(humanoid_->thormang3_link_data_[manipulation_module_state_->ik_id_end_]->position_.coeff(1, 0)
                              - manipulation_module_state_->goal_kinematics_pose_msg_.pose.position.y, 2)
                        + pow(humanoid_->thormang3_link_data_[manipulation_module_state_->ik_id_end_]->position_.coeff(2, 0)
                              - manipulation_module_state_->goal_kinematics_pose_msg_.pose.position.z, 2)
                      );

    manipulation_module_state_->mov_time_ = diff / tol;
    int all_time_steps = int(floor((manipulation_module_state_->mov_time_ / manipulation_module_state_->smp_time_) + 1.0));
    manipulation_module_state_->mov_time_ = double(all_time_steps - 1) * manipulation_module_state_->smp_time_;

    if (manipulation_module_state_->mov_time_ < mov_time)
      manipulation_module_state_->mov_time_ = mov_time;
  }
  else
  {
    manipulation_module_state_->mov_time_ = manipulation_module_state_->goal_joint_pose_msg_.time;
  }

  manipulation_module_state_->all_time_steps_ = int(manipulation_module_state_->mov_time_ / manipulation_module_state_->smp_time_) + 1;
  manipulation_module_state_->calc_task_tra_.resize(manipulation_module_state_->all_time_steps_, 3);

  /* calculate trajectory */
  for (int dim = 0; dim < 3; dim++)
  {
    double ini_value = humanoid_->thormang3_link_data_[manipulation_module_state_->ik_id_end_]->position_.coeff(dim, 0);
    double tar_value;
    if (dim == 0)
      tar_value = manipulation_module_state_->goal_kinematics_pose_msg_.pose.position.x;
    else if (dim == 1)
      tar_value = manipulation_module_state_->goal_kinematics_pose_msg_.pose.position.y;
    else if (dim == 2)
      tar_value = manipulation_module_state_->goal_kinematics_pose_msg_.pose.position.z;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                manipulation_module_state_->smp_time_,
                                                                manipulation_module_state_->mov_time_);

    manipulation_module_state_->calc_task_tra_.block(0, dim, manipulation_module_state_->all_time_steps_, 1) = tra;
  }

  manipulation_module_state_->cnt_        = 0;
  manipulation_module_state_->is_moving_  = true;
  manipulation_module_state_->ik_solve_   = true;

  ROS_INFO("[start] send trajectory");
}

void ManipulationModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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

    joint_state_->curr_joint_state[joint_name_to_id[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state[joint_name_to_id[joint_name]].position_ = joint_goal_position;
  }

  /*----- forward kinematics -----*/

  for (int id = 1; id <= MAX_JOINT_ID; id++)
    humanoid_->thormang3_link_data_[id]->joint_angle_ = joint_state_->goal_joint_state[id].position_;

  humanoid_->calcForwardKinematics(0);

  /* ----- send trajectory ----- */

  if (manipulation_module_state_->is_moving_ == true)
  {
    if (manipulation_module_state_->cnt_ == 0)
    {
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

      manipulation_module_state_->ik_start_rotation_ =
                  humanoid_->thormang3_link_data_[manipulation_module_state_->ik_id_end_]->orientation_;
    }

    if (manipulation_module_state_->ik_solve_ == true)
    {
      /* ----- inverse kinematics ----- */
      manipulation_module_state_->setInverseKinematics(manipulation_module_state_->cnt_,
                                                       manipulation_module_state_->ik_start_rotation_);

      int     max_iter    = 30;
      double  ik_tol      = 1e-3;
      bool    ik_success  = humanoid_->calcInverseKinematics(manipulation_module_state_->ik_id_start_,
                                                             manipulation_module_state_->ik_id_end_,
                                                             manipulation_module_state_->ik_target_position_,
                                                             manipulation_module_state_->ik_target_rotation_,
                                                             max_iter, ik_tol,
                                                             manipulation_module_state_->ik_weight_);

      if (ik_success == true)
      {
        for (int id = 1; id <= MAX_JOINT_ID; id++)
          joint_state_->goal_joint_state[id].position_ = humanoid_->thormang3_link_data_[id]->joint_angle_;
      }
      else
      {
        ROS_INFO("----- ik failed -----");
        ROS_INFO("[end] send trajectory");

        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

        manipulation_module_state_->is_moving_  = false;
        manipulation_module_state_->ik_solve_   = false;
        manipulation_module_state_->cnt_        = 0;
      }
    }
    else
    {
      for (int id = 1; id <= MAX_JOINT_ID; id++)
        joint_state_->goal_joint_state[id].position_ = manipulation_module_state_->calc_joint_tra_(manipulation_module_state_->cnt_, id);
    }

    manipulation_module_state_->cnt_++;
  }

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state[joint_name_to_id[joint_name]].position_;
  }

  /*---------- initialize count number ----------*/
  if (manipulation_module_state_->is_moving_ == true)
  {
    if (manipulation_module_state_->cnt_ >= manipulation_module_state_->all_time_steps_)
    {
      ROS_INFO("[end] send trajectory");

      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      manipulation_module_state_->is_moving_  = false;
      manipulation_module_state_->ik_solve_   = false;
      manipulation_module_state_->cnt_        = 0;
    }
  }

}

void ManipulationModule::stop()
{
  manipulation_module_state_->is_moving_  = false;
  manipulation_module_state_->ik_solve_   = false;
  manipulation_module_state_->cnt_        = 0;

  return;
}

bool ManipulationModule::isRunning()
{
  return manipulation_module_state_->is_moving_;
}

void ManipulationModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Manipulation";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}
