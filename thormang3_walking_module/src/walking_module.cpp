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
 * wakling_module.cpp
 *
 *  Created on: 2016. 1. 25.
 *      Author: Jay Song
 */

#include <stdio.h>
#include <eigen_conversions/eigen_msg.h>

#include "thormang3_walking_module/walking_module.h"

using namespace thormang3;

class WalkingStatusMSG
{
public:
  static const std::string FAILED_TO_ADD_STEP_DATA_MSG;
  static const std::string BALANCE_PARAM_SETTING_START_MSG;
  static const std::string BALANCE_PARAM_SETTING_FINISH_MSG;
  static const std::string WALKING_MODULE_IS_ENABLED_MSG;
  static const std::string WALKING_MODULE_IS_DISABLED_MSG;
  static const std::string WALKING_START_MSG;
  static const std::string WALKING_FINISH_MSG;
};

const std::string WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG = "Failed_to_add_Step_Data";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_START_MSG = "Balance_Param_Setting_Started";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG = "Balance_Param_Setting_Finished";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG = "Walking_Module_is_enabled";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG = "Walking_Module_is_disabled";
const std::string WalkingStatusMSG::WALKING_START_MSG = "Walking_Started";
const std::string WalkingStatusMSG::WALKING_FINISH_MSG = "Walking_Finished";


WalkingMotionModule::WalkingMotionModule()
: control_cycle_msec_(8)
{
  enable_          = false;
  module_name_     = "walking_module";
  control_mode_    = robotis_framework::PositionControl;
  result_["r_leg_hip_y"] = new robotis_framework::DynamixelState();
  result_["r_leg_hip_r"] = new robotis_framework::DynamixelState();
  result_["r_leg_hip_p"] = new robotis_framework::DynamixelState();
  result_["r_leg_kn_p" ] = new robotis_framework::DynamixelState();
  result_["r_leg_an_p" ] = new robotis_framework::DynamixelState();
  result_["r_leg_an_r" ] = new robotis_framework::DynamixelState();

  result_["l_leg_hip_y"] = new robotis_framework::DynamixelState();
  result_["l_leg_hip_r"] = new robotis_framework::DynamixelState();
  result_["l_leg_hip_p"] = new robotis_framework::DynamixelState();
  result_["l_leg_kn_p" ] = new robotis_framework::DynamixelState();
  result_["l_leg_an_p" ] = new robotis_framework::DynamixelState();
  result_["l_leg_an_r" ] = new robotis_framework::DynamixelState();

  previous_running_    = present_running    = false;

  gyro_roll_ =  gyro_pitch_ = 0;
  orientation_roll_ = orientation_pitch_ = 0;
  r_foot_fx_N_  = r_foot_fy_N_  = r_foot_fz_N_  = 0;
  r_foot_Tx_Nm_ = r_foot_Ty_Nm_ = r_foot_Tz_Nm_ = 0;
  l_foot_fx_N_  = l_foot_fy_N_  = l_foot_fz_N_  = 0;
  l_foot_Tx_Nm_ = l_foot_Ty_Nm_ = l_foot_Tz_Nm_ = 0;


  r_foot_ft_publish_checker_ = -1;
  l_foot_ft_publish_checker_ = -1;

  desired_matrix_g_to_cob_   = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_rfoot_ = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_lfoot_ = Eigen::MatrixXd::Identity(4,4);


  balance_update_with_loop_ = false;
  balance_update_duration_ = 1.0;
  balance_update_sys_time_ = 1.0;
  balance_update_polynomial_coeff_.resize(6, 1);

  double tf = balance_update_duration_;
  Eigen::MatrixXd A(6,6), B(6, 1);
  A <<  0.0,     0.0,     0.0,    0.0,    0.0, 1.0,
      0.0,   0.0,    0.0,    0.0,       1.0, 0.0,
      0.0,   0.0,    0.0,    2.0,       0.0, 0.0,
      tf*tf*tf*tf*tf,     tf*tf*tf*tf,      tf*tf*tf,        tf*tf,     tf, 1.0,
      5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,    2.0*tf,        1.0, 0.0,
      20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,        2.0,        0.0, 0.0;

  B << 0, 0, 0, 1.0, 0, 0;

  balance_update_polynomial_coeff_ = A.inverse() * B;

  rot_x_pi_3d_.resize(3,3);
  rot_x_pi_3d_ << 1,  0,  0,
                  0, -1,  0,
                  0,  0, -1;
  rot_z_pi_3d_.resize(3,3);
  rot_z_pi_3d_ << -1,  0, 0,
                   0, -1, 0,
                   0,  0, 1;
}

WalkingMotionModule::~WalkingMotionModule()
{
  queue_thread_.join();
}

void WalkingMotionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&WalkingMotionModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;

  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();
  online_walking->setInitialPose(0, -0.093, -0.63, 0, 0, 0,
                                 0,  0.093, -0.63, 0, 0, 0,
                                 0,      0,     0, 0, 0, 0);


  online_walking->hip_roll_feedforward_angle_rad_ = 0.0*M_PI/180.0;
  online_walking->balance_ctrl_.setCOBManualAdjustment(-10.0*0.001, 0, 0);

  online_walking->initialize();

  publish_mutex_.lock();
  desired_matrix_g_to_cob_   = online_walking->mat_g_to_cob_;
  desired_matrix_g_to_rfoot_ = online_walking->mat_g_to_rfoot_;
  desired_matrix_g_to_lfoot_ = online_walking->mat_g_to_lfoot_;
  publish_mutex_.unlock();

  result_["r_leg_hip_y"]->goal_position_ = online_walking->out_angle_rad_[0];
  result_["r_leg_hip_r"]->goal_position_ = online_walking->out_angle_rad_[1];
  result_["r_leg_hip_p"]->goal_position_ = online_walking->out_angle_rad_[2];
  result_["r_leg_kn_p"]->goal_position_  = online_walking->out_angle_rad_[3];
  result_["r_leg_an_p"]->goal_position_  = online_walking->out_angle_rad_[4];
  result_["r_leg_an_r"]->goal_position_  = online_walking->out_angle_rad_[5];

  result_["l_leg_hip_y"]->goal_position_ = online_walking->out_angle_rad_[6];
  result_["l_leg_hip_r"]->goal_position_ = online_walking->out_angle_rad_[7];
  result_["l_leg_hip_p"]->goal_position_ = online_walking->out_angle_rad_[8];
  result_["l_leg_kn_p" ]->goal_position_ = online_walking->out_angle_rad_[9];
  result_["l_leg_an_p" ]->goal_position_ = online_walking->out_angle_rad_[10];
  result_["l_leg_an_r" ]->goal_position_ = online_walking->out_angle_rad_[11];

  online_walking->start();
  online_walking->process();

  previous_running_ = isRunning();

  online_walking->hip_roll_feedforward_angle_rad_ = 0.0;

  online_walking->balance_ctrl_.setGyroBalanceGainRatio(0.0);

  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_  = 0.0;
  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_ = 0.0;

  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_ = 0.0;
  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_ = 0.0;
  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.gain_  = 0.0;
  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.gain_  = 0.0;

  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_ = 0.0;

  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_  = 0.0;
  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_ = 0.0;
  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.gain_   = 0.0;
  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.gain_  = 0.0;
}

void    WalkingMotionModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  robot_pose_pub_ = ros_node.advertise<thormang3_walking_module_msgs::RobotPose>("/robotis/walking/robot_pose", 1);
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);


  /* ROS Service Callback Functions */
  ros::ServiceServer get_ref_step_data_server  = ros_node.advertiseService("/robotis/walking/get_reference_step_data",   &WalkingMotionModule::getReferenceStepDataServiceCallback,   this);
  ros::ServiceServer add_step_data_array_sever = ros_node.advertiseService("/robotis/walking/add_step_data",             &WalkingMotionModule::addStepDataServiceCallback,            this);
  ros::ServiceServer walking_start_server      = ros_node.advertiseService("/robotis/walking/walking_start",             &WalkingMotionModule::startWalkingServiceCallback,           this);
  ros::ServiceServer is_running_server         = ros_node.advertiseService("/robotis/walking/is_running",                &WalkingMotionModule::IsRunningServiceCallback,              this);
  ros::ServiceServer set_balance_param_server  = ros_node.advertiseService("/robotis/walking/set_balance_param",         &WalkingMotionModule::setBalanceParamServiceCallback,        this);
  ros::ServiceServer remove_existing_step_data = ros_node.advertiseService("/robotis/walking/remove_existing_step_data", &WalkingMotionModule::removeExistingStepDataServiceCallback, this);

  /* sensor topic subscribe */
  ros::Subscriber imu_data_sub    = ros_node.subscribe("/robotis/sensor/imu/imu",    3, &WalkingMotionModule::imuDataOutputCallback,        this);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void WalkingMotionModule::publishRobotPose(void)
{
  publish_mutex_.lock();
  robot_pose_msg_.global_to_center_of_body.position.x = desired_matrix_g_to_cob_.coeff(0, 3);
  robot_pose_msg_.global_to_center_of_body.position.y = desired_matrix_g_to_cob_.coeff(1, 3);
  robot_pose_msg_.global_to_center_of_body.position.z = desired_matrix_g_to_cob_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_cob(desired_matrix_g_to_cob_.block<3, 3>(0, 0));


  robot_pose_msg_.global_to_right_foot.position.x = desired_matrix_g_to_rfoot_.coeff(0, 3);
  robot_pose_msg_.global_to_right_foot.position.y = desired_matrix_g_to_rfoot_.coeff(1, 3);
  robot_pose_msg_.global_to_right_foot.position.z = desired_matrix_g_to_rfoot_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_rf(desired_matrix_g_to_rfoot_.block<3, 3>(0, 0));

  robot_pose_msg_.global_to_left_foot.position.x = desired_matrix_g_to_lfoot_.coeff(0, 3);
  robot_pose_msg_.global_to_left_foot.position.y = desired_matrix_g_to_lfoot_.coeff(1, 3);
  robot_pose_msg_.global_to_left_foot.position.z = desired_matrix_g_to_lfoot_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_lf(desired_matrix_g_to_lfoot_.block<3, 3>(0, 0));
  publish_mutex_.unlock();

  tf::quaternionEigenToMsg(quaterniond_g_to_cob, robot_pose_msg_.global_to_center_of_body.orientation);
  tf::quaternionEigenToMsg(quaterniond_g_to_rf,  robot_pose_msg_.global_to_right_foot.orientation);
  tf::quaternionEigenToMsg(quaterniond_g_to_lf,  robot_pose_msg_.global_to_left_foot.orientation);

  robot_pose_pub_.publish(robot_pose_msg_);

}

void WalkingMotionModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

int WalkingMotionModule::convertStepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, robotis_framework::StepData& des)
{
  int copy_result = thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;
  des.time_data.   walking_state           = src.time_data.walking_state;
  des.time_data.abs_step_time            = src.time_data.abs_step_time;
  des.time_data.dsp_ratio               = src.time_data.dsp_ratio;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.shoulder_swing_gain = 0;
  des.position_data.elbow_swing_gain    = 0;
  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.waist_pitch_angle   = 0;
  des.position_data.waist_yaw_angle     = src.position_data.torso_yaw_angle_rad;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.body_pose.z          = src.position_data.body_pose.z;
  des.position_data.body_pose.roll       = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch      = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw        = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  des.time_data.start_time_delay_ratio_x         = src.time_data.start_time_delay_ratio_x;
  des.time_data.start_time_delay_ratio_y         = src.time_data.start_time_delay_ratio_y;
  des.time_data.start_time_delay_ratio_z         = src.time_data.start_time_delay_ratio_z;
  des.time_data.start_time_delay_ratio_roll      = src.time_data.start_time_delay_ratio_roll;
  des.time_data.start_time_delay_ratio_pitch     = src.time_data.start_time_delay_ratio_pitch;
  des.time_data.start_time_delay_ratio_yaw       = src.time_data.start_time_delay_ratio_yaw;

  des.time_data.finish_time_advance_ratio_x     = src.time_data.finish_time_advance_ratio_x;
  des.time_data.finish_time_advance_ratio_y     = src.time_data.finish_time_advance_ratio_y;
  des.time_data.finish_time_advance_ratio_z     = src.time_data.finish_time_advance_ratio_z;
  des.time_data.finish_time_advance_ratio_roll  = src.time_data.finish_time_advance_ratio_roll;
  des.time_data.finish_time_advance_ratio_pitch = src.time_data.finish_time_advance_ratio_pitch;
  des.time_data.finish_time_advance_ratio_yaw   = src.time_data.finish_time_advance_ratio_yaw;

  if((src.time_data.walking_state != thormang3_walking_module_msgs::StepTimeData::IN_WALKING_STARTING)
      && (src.time_data.walking_state != thormang3_walking_module_msgs::StepTimeData::IN_WALKING)
      && (src.time_data.walking_state != thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.start_time_delay_ratio_x     < 0)
      || (src.time_data.start_time_delay_ratio_y     < 0)
      || (src.time_data.start_time_delay_ratio_z     < 0)
      || (src.time_data.start_time_delay_ratio_roll  < 0)
      || (src.time_data.start_time_delay_ratio_pitch < 0)
      || (src.time_data.start_time_delay_ratio_yaw   < 0) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.finish_time_advance_ratio_x     < 0)
      || (src.time_data.finish_time_advance_ratio_y     < 0)
      || (src.time_data.finish_time_advance_ratio_z     < 0)
      || (src.time_data.finish_time_advance_ratio_roll  < 0)
      || (src.time_data.finish_time_advance_ratio_pitch < 0)
      || (src.time_data.finish_time_advance_ratio_yaw   < 0) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
      || ((src.time_data.start_time_delay_ratio_y      + src.time_data.finish_time_advance_ratio_y     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_z      + src.time_data.finish_time_advance_ratio_z     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_roll   + src.time_data.finish_time_advance_ratio_roll  ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_pitch  + src.time_data.finish_time_advance_ratio_pitch ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_yaw    + src.time_data.finish_time_advance_ratio_yaw   ) > 1.0) )
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.position_data.moving_foot != thormang3_walking_module_msgs::StepPositionData::STANDING)
      && (src.position_data.moving_foot != thormang3_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
      && (src.position_data.moving_foot != thormang3_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING))
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  if(src.position_data.foot_z_swap < 0)
    copy_result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  return copy_result;
}

int WalkingMotionModule::convertStepDataToStepDataMsg(robotis_framework::StepData& src, thormang3_walking_module_msgs::StepData& des)
{
  des.time_data.walking_state   = src.time_data.   walking_state;
  des.time_data.abs_step_time   = src.time_data.abs_step_time;
  des.time_data.dsp_ratio       = src.time_data.dsp_ratio;

  des.time_data.start_time_delay_ratio_x     = des.time_data.finish_time_advance_ratio_x     = 0;
  des.time_data.start_time_delay_ratio_y     = des.time_data.finish_time_advance_ratio_y     = 0;
  des.time_data.start_time_delay_ratio_z     = des.time_data.finish_time_advance_ratio_z     = 0;
  des.time_data.start_time_delay_ratio_roll  = des.time_data.finish_time_advance_ratio_roll  = 0;
  des.time_data.start_time_delay_ratio_pitch = des.time_data.finish_time_advance_ratio_pitch = 0;
  des.time_data.start_time_delay_ratio_yaw   = des.time_data.finish_time_advance_ratio_yaw   = 0;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.torso_yaw_angle_rad = src.position_data.waist_yaw_angle;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.body_pose.z           = src.position_data.body_pose.z;
  des.position_data.body_pose.roll        = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch       = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw         = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  return 0;
}

bool WalkingMotionModule::getReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request &req,
                                                              thormang3_walking_module_msgs::GetReferenceStepData::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  robotis_framework::StepData refStepData;

  online_walking->getReferenceStepDatafotAddition(&refStepData);

  convertStepDataToStepDataMsg(refStepData, res.reference_step_data);

  return true;
}

bool WalkingMotionModule::addStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request &req,
                                                     thormang3_walking_module_msgs::AddStepDataArray::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();
  res.result = thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE;
    std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if((req.step_data_array.size() > 100)
      && (req.remove_existing_step_data == true)
      && ((online_walking->isRunning() == true)))
  {
    res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::TOO_MANY_STEP_DATA;
    std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  robotis_framework::StepData step_data, ref_step_data;
  std::vector<robotis_framework::StepData> req_step_data_array;

  online_walking->getReferenceStepDatafotAddition(&ref_step_data);

  for(int i = 0; i < req.step_data_array.size(); i++)
  {
    res.result |= convertStepDataMsgToStepData(req.step_data_array[i], step_data);

    if(step_data.time_data.abs_step_time <= 0)
    {
      res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
    }

    if(i != 0)
    {
      if(step_data.time_data.abs_step_time <= req_step_data_array[req_step_data_array.size() - 1].time_data.abs_step_time)
      {
        res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }
    else
    {
      if(step_data.time_data.abs_step_time <= ref_step_data.time_data.abs_step_time)
      {
        res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }

    if(res.result != thormang3_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
    {
      std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }

    req_step_data_array.push_back(step_data);
  }


  if(req.remove_existing_step_data == true)
  {
    int exist_num_of_step_data = online_walking->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        online_walking->eraseLastStepData();
  }
  else
  {
    if(online_walking->isRunning() == true)
    {
      res.result |= thormang3_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW;
      std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }
  }

  for(unsigned int i = 0; i < req_step_data_array.size() ; i++)
    online_walking->addStepData(req_step_data_array[i]);

  if( req.auto_start == true)
  {
    online_walking->start();
  }

  return true;
}

bool WalkingMotionModule::startWalkingServiceCallback(thormang3_walking_module_msgs::StartWalking::Request &req,
    thormang3_walking_module_msgs::StartWalking::Response &res)
{
  RobotisOnlineWalking *prev_walking = RobotisOnlineWalking::getInstance();
  res.result = thormang3_walking_module_msgs::StartWalking::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= thormang3_walking_module_msgs::StartWalking::Response::NOT_ENABLED_WALKING_MODULE;
  }

  if(prev_walking->isRunning() == true)
  {
    res.result |= thormang3_walking_module_msgs::StartWalking::Response::ROBOT_IS_WALKING_NOW;
  }

  if(prev_walking->getNumofRemainingUnreservedStepData() == 0)
  {
    res.result |= thormang3_walking_module_msgs::StartWalking::Response::NO_STEP_DATA;
  }

  if(res.result == thormang3_walking_module_msgs::StartWalking::Response::NO_ERROR)
  {
    prev_walking->start();
  }

  return true;
}

bool WalkingMotionModule::IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request &req,
    thormang3_walking_module_msgs::IsRunning::Response &res)
{
  bool is_running = isRunning();
  res.is_running = is_running;

  return true;
}

bool WalkingMotionModule::isRunning()
{
  return RobotisOnlineWalking::getInstance()->isRunning();
}

bool WalkingMotionModule::setBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
                                                         thormang3_walking_module_msgs::SetBalanceParam::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();
  res.result = thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR;

  if( enable_ == false)
    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE;

  if( balance_update_with_loop_ == true)
  {
    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED;
  }

  if( ( req.balance_param.foot_roll_angle_time_constant  <= 0.0 )
      || ( req.balance_param.foot_pitch_angle_time_constant  <= 0.0 )
      || ( req.balance_param.foot_x_force_time_constant      <= 0.0 )
      || ( req.balance_param.foot_y_force_time_constant      <= 0.0 )
      || ( req.balance_param.foot_z_force_time_constant      <= 0.0 )
      || ( req.balance_param.foot_roll_torque_time_constant  <= 0.0 )
      || ( req.balance_param.foot_pitch_torque_time_constant <= 0.0 ) )
  {
    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::TIME_CONST_IS_ZERO_OR_NEGATIVE;
  }

  if(res.result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
  {
    if( req.updating_duration < 0.0 )
    {
      // under 8ms apply immediately
      setBalanceParam(req.balance_param);
      std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      return true;
    }
    else
    {
      balance_update_duration_ = req.updating_duration;
    }

    balance_update_sys_time_ = 0.0;
    balance_update_polynomial_coeff_.resize(6, 1);

    double tf = balance_update_duration_;
    Eigen::MatrixXd A(6,6), B(6, 1);
    A <<  0.0,     0.0,     0.0,    0.0,    0.0, 1.0,
        0.0,   0.0,    0.0,    0.0,       1.0, 0.0,
        0.0,   0.0,    0.0,    2.0,       0.0, 0.0,
        tf*tf*tf*tf*tf,     tf*tf*tf*tf,      tf*tf*tf,        tf*tf,     tf, 1.0,
        5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,    2.0*tf,        1.0, 0.0,
        20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,        2.0,        0.0, 0.0;

    B << 0, 0, 0, 1.0, 0, 0;
    balance_update_polynomial_coeff_ = A.inverse() * B;

    desired_balance_param_ = req.balance_param;

    previous_balance_param_.cob_x_offset_m                  = online_walking->balance_ctrl_.getCOBManualAdjustmentX();
    previous_balance_param_.cob_y_offset_m                  = online_walking->balance_ctrl_.getCOBManualAdjustmentY();

    previous_balance_param_.hip_roll_swap_angle_rad         = online_walking->hip_roll_feedforward_angle_rad_;

    previous_balance_param_.gyro_gain                       = online_walking->balance_ctrl_.getGyroBalanceGainRatio();
    previous_balance_param_.foot_roll_angle_gain            = online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_;
    previous_balance_param_.foot_pitch_angle_gain           = online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_;

    previous_balance_param_.foot_x_force_gain               = online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_;
    previous_balance_param_.foot_y_force_gain               = online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_;
    previous_balance_param_.foot_z_force_gain               = online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_;
    previous_balance_param_.foot_roll_torque_gain           = online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_;
    previous_balance_param_.foot_pitch_torque_gain          = online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_;

    previous_balance_param_.foot_roll_angle_time_constant   = online_walking->balance_ctrl_.foot_roll_angle_ctrl_.time_constant_sec_;
    previous_balance_param_.foot_pitch_angle_time_constant  = online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.time_constant_sec_;

    previous_balance_param_.foot_x_force_time_constant      = online_walking->balance_ctrl_.right_foot_force_x_ctrl_.time_constant_sec_;
    previous_balance_param_.foot_y_force_time_constant      = online_walking->balance_ctrl_.right_foot_force_y_ctrl_.time_constant_sec_;
    previous_balance_param_.foot_z_force_time_constant      = online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.time_constant_sec_;
    previous_balance_param_.foot_roll_torque_time_constant  = online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.time_constant_sec_;
    previous_balance_param_.foot_pitch_torque_time_constant = online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.time_constant_sec_;

    balance_update_with_loop_ = true;

    std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_START_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }

  return true;
}

void WalkingMotionModule::setBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  online_walking->hip_roll_feedforward_angle_rad_ = balance_param_msg.hip_roll_swap_angle_rad;

  online_walking->balance_ctrl_.setCOBManualAdjustment(balance_param_msg.cob_x_offset_m, balance_param_msg.cob_y_offset_m, 0);

  // set gain
  online_walking->balance_ctrl_.setGyroBalanceGainRatio(balance_param_msg.gyro_gain);

  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_  = balance_param_msg.foot_roll_angle_gain;
  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_ = balance_param_msg.foot_pitch_angle_gain;

  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_ = balance_param_msg.foot_x_force_gain;
  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_ = balance_param_msg.foot_y_force_gain;
  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.gain_  = balance_param_msg.foot_x_force_gain;
  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.gain_  = balance_param_msg.foot_y_force_gain;

  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_ = balance_param_msg.foot_z_force_gain;

  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_  = balance_param_msg.foot_roll_torque_gain;
  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_ = balance_param_msg.foot_roll_torque_gain;
  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.gain_   = balance_param_msg.foot_pitch_torque_gain;
  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.gain_  = balance_param_msg.foot_pitch_torque_gain;

  // set time_const
  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.time_constant_sec_  = balance_param_msg.foot_roll_angle_time_constant;
  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.time_constant_sec_ = balance_param_msg.foot_pitch_angle_time_constant;

  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.time_constant_sec_ = balance_param_msg.foot_x_force_time_constant;
  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.time_constant_sec_ = balance_param_msg.foot_y_force_time_constant;
  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.time_constant_sec_  = balance_param_msg.foot_x_force_time_constant;
  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.time_constant_sec_  = balance_param_msg.foot_y_force_time_constant;

  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.time_constant_sec_ = balance_param_msg.foot_z_force_time_constant;

  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.time_constant_sec_  = balance_param_msg.foot_roll_torque_time_constant;
  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.time_constant_sec_ = balance_param_msg.foot_pitch_torque_time_constant;
  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.time_constant_sec_   = balance_param_msg.foot_roll_torque_time_constant;
  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.time_constant_sec_  = balance_param_msg.foot_pitch_torque_time_constant;
}

bool WalkingMotionModule::removeExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
                                                                thormang3_walking_module_msgs::RemoveExistingStepData::Response &res)
{
  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  res.result = thormang3_walking_module_msgs::RemoveExistingStepData::Response::NO_ERROR;

  if(isRunning())
  {
    res.result |= thormang3_walking_module_msgs::RemoveExistingStepData::Response::ROBOT_IS_WALKING_NOW;
  }
  else
  {
    int exist_num_of_step_data = online_walking->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        online_walking->eraseLastStepData();
  }
  return true;
}


void WalkingMotionModule::imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  RobotisOnlineWalking::getInstance()->current_gyro_roll_rad_per_sec_  = -1.0*(msg->angular_velocity.x);
  RobotisOnlineWalking::getInstance()->current_gyro_pitch_rad_per_sec_ = -1.0*(msg->angular_velocity.y);

  Eigen::Quaterniond imu_quat;
  tf::quaternionMsgToEigen(msg->orientation, RobotisOnlineWalking::getInstance()->quat_current_imu_);
}

void WalkingMotionModule::onModuleEnable()
{
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void WalkingMotionModule::onModuleDisable()
{
  previous_running_ = present_running = false;

  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG;
  balance_update_with_loop_ = false;
  online_walking->hip_roll_feedforward_angle_rad_ = 0.0;

  online_walking->balance_ctrl_.setGyroBalanceGainRatio(0.0);

  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_  = 0.0;
  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_ = 0.0;

  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_ = 0.0;
  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_ = 0.0;
  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.gain_  = 0.0;
  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.gain_  = 0.0;

  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_ = 0.0;

  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_  = 0.0;
  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_ = 0.0;
  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.gain_   = 0.0;
  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.gain_  = 0.0;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void WalkingMotionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if(enable_ == false)
    return;

  RobotisOnlineWalking *online_walking = RobotisOnlineWalking::getInstance();

  r_foot_fx_N_  = sensors["r_foot_fx_scaled_N"];
  r_foot_fy_N_  = sensors["r_foot_fy_scaled_N"];
  r_foot_fz_N_  = sensors["r_foot_fz_scaled_N"];
  r_foot_Tx_Nm_ = sensors["r_foot_tx_scaled_Nm"];
  r_foot_Ty_Nm_ = sensors["r_foot_ty_scaled_Nm"];
  r_foot_Tz_Nm_ = sensors["r_foot_tz_scaled_Nm"];

  l_foot_fx_N_  = sensors["l_foot_fx_scaled_N"];
  l_foot_fy_N_  = sensors["l_foot_fy_scaled_N"];
  l_foot_fz_N_  = sensors["l_foot_fz_scaled_N"];
  l_foot_Tx_Nm_ = sensors["l_foot_tx_scaled_Nm"];
  l_foot_Ty_Nm_ = sensors["l_foot_ty_scaled_Nm"];
  l_foot_Tz_Nm_ = sensors["l_foot_tz_scaled_Nm"];


  r_foot_fx_N_ = robotis_framework::sign(r_foot_fx_N_) * fmin( fabs(r_foot_fx_N_), 2000.0);
  r_foot_fy_N_ = robotis_framework::sign(r_foot_fy_N_) * fmin( fabs(r_foot_fy_N_), 2000.0);
  r_foot_fz_N_ = robotis_framework::sign(r_foot_fz_N_) * fmin( fabs(r_foot_fz_N_), 2000.0);
  r_foot_Tx_Nm_ = robotis_framework::sign(r_foot_Tx_Nm_) *fmin(fabs(r_foot_Tx_Nm_), 300.0);
  r_foot_Ty_Nm_ = robotis_framework::sign(r_foot_Ty_Nm_) *fmin(fabs(r_foot_Ty_Nm_), 300.0);
  r_foot_Tz_Nm_ = robotis_framework::sign(r_foot_Tz_Nm_) *fmin(fabs(r_foot_Tz_Nm_), 300.0);

  l_foot_fx_N_ = robotis_framework::sign(l_foot_fx_N_) * fmin( fabs(l_foot_fx_N_), 2000.0);
  l_foot_fy_N_ = robotis_framework::sign(l_foot_fy_N_) * fmin( fabs(l_foot_fy_N_), 2000.0);
  l_foot_fz_N_ = robotis_framework::sign(l_foot_fz_N_) * fmin( fabs(l_foot_fz_N_), 2000.0);
  l_foot_Tx_Nm_ = robotis_framework::sign(l_foot_Tx_Nm_) *fmin(fabs(l_foot_Tx_Nm_), 300.0);
  l_foot_Ty_Nm_ = robotis_framework::sign(l_foot_Ty_Nm_) *fmin(fabs(l_foot_Ty_Nm_), 300.0);
  l_foot_Tz_Nm_ = robotis_framework::sign(l_foot_Tz_Nm_) *fmin(fabs(l_foot_Tz_Nm_), 300.0);


  if(balance_update_with_loop_ == true)
  {
    balance_update_sys_time_ += control_cycle_msec_ * 0.001;
    if(balance_update_sys_time_ >= balance_update_duration_ )
    {
      balance_update_sys_time_ = balance_update_duration_;
      balance_update_with_loop_ = false;
      setBalanceParam(desired_balance_param_);
      std::string _status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
    }
    else
    {
      double current_update_gain =  balance_update_polynomial_coeff_.coeff(0,0) * robotis_framework::powDI(balance_update_sys_time_ , 5)
      + balance_update_polynomial_coeff_.coeff(1,0) * robotis_framework::powDI(balance_update_sys_time_ , 4)
      + balance_update_polynomial_coeff_.coeff(2,0) * robotis_framework::powDI(balance_update_sys_time_ , 3)
      + balance_update_polynomial_coeff_.coeff(3,0) * robotis_framework::powDI(balance_update_sys_time_ , 2)
      + balance_update_polynomial_coeff_.coeff(4,0) * robotis_framework::powDI(balance_update_sys_time_ , 1)
      + balance_update_polynomial_coeff_.coeff(5,0) ;

      current_balance_param_.cob_x_offset_m                  = current_update_gain*(desired_balance_param_.cob_x_offset_m                   - previous_balance_param_.cob_x_offset_m                ) + previous_balance_param_.cob_x_offset_m;
      current_balance_param_.cob_y_offset_m                  = current_update_gain*(desired_balance_param_.cob_y_offset_m                   - previous_balance_param_.cob_y_offset_m                ) + previous_balance_param_.cob_y_offset_m;

      current_balance_param_.hip_roll_swap_angle_rad         = current_update_gain*(desired_balance_param_.hip_roll_swap_angle_rad         - previous_balance_param_.hip_roll_swap_angle_rad        ) + previous_balance_param_.hip_roll_swap_angle_rad;

      current_balance_param_.gyro_gain                       = current_update_gain*(desired_balance_param_.gyro_gain                         - previous_balance_param_.gyro_gain                    ) + previous_balance_param_.gyro_gain;
      current_balance_param_.foot_roll_angle_gain            = current_update_gain*(desired_balance_param_.foot_roll_angle_gain            - previous_balance_param_.foot_roll_angle_gain           ) + previous_balance_param_.foot_roll_angle_gain;
      current_balance_param_.foot_pitch_angle_gain           = current_update_gain*(desired_balance_param_.foot_pitch_angle_gain           - previous_balance_param_.foot_pitch_angle_gain          ) + previous_balance_param_.foot_pitch_angle_gain;

      current_balance_param_.foot_x_force_gain               = current_update_gain*(desired_balance_param_.foot_x_force_gain               - previous_balance_param_.foot_x_force_gain              ) + previous_balance_param_.foot_x_force_gain;
      current_balance_param_.foot_y_force_gain               = current_update_gain*(desired_balance_param_.foot_y_force_gain               - previous_balance_param_.foot_y_force_gain              ) + previous_balance_param_.foot_y_force_gain;
      current_balance_param_.foot_z_force_gain               = current_update_gain*(desired_balance_param_.foot_z_force_gain               - previous_balance_param_.foot_z_force_gain              ) + previous_balance_param_.foot_z_force_gain;
      current_balance_param_.foot_roll_torque_gain           = current_update_gain*(desired_balance_param_.foot_roll_torque_gain           - previous_balance_param_.foot_roll_torque_gain          ) + previous_balance_param_.foot_roll_torque_gain;
      current_balance_param_.foot_pitch_torque_gain          = current_update_gain*(desired_balance_param_.foot_pitch_torque_gain          - previous_balance_param_.foot_pitch_torque_gain         ) + previous_balance_param_.foot_pitch_torque_gain;

      current_balance_param_.foot_roll_angle_time_constant   = current_update_gain*(desired_balance_param_.foot_roll_angle_time_constant   - previous_balance_param_.foot_roll_angle_time_constant  ) + previous_balance_param_.foot_roll_angle_time_constant;
      current_balance_param_.foot_pitch_angle_time_constant  = current_update_gain*(desired_balance_param_.foot_pitch_angle_time_constant  - previous_balance_param_.foot_pitch_angle_time_constant ) + previous_balance_param_.foot_pitch_angle_time_constant;

      current_balance_param_.foot_x_force_time_constant      = current_update_gain*(desired_balance_param_.foot_x_force_time_constant      - previous_balance_param_.foot_x_force_time_constant     ) + previous_balance_param_.foot_x_force_time_constant;
      current_balance_param_.foot_y_force_time_constant      = current_update_gain*(desired_balance_param_.foot_y_force_time_constant      - previous_balance_param_.foot_y_force_time_constant     ) + previous_balance_param_.foot_y_force_time_constant;
      current_balance_param_.foot_z_force_time_constant      = current_update_gain*(desired_balance_param_.foot_z_force_time_constant      - previous_balance_param_.foot_z_force_time_constant     ) + previous_balance_param_.foot_z_force_time_constant;
      current_balance_param_.foot_roll_torque_time_constant  = current_update_gain*(desired_balance_param_.foot_roll_torque_time_constant  - previous_balance_param_.foot_roll_torque_time_constant ) + previous_balance_param_.foot_roll_torque_time_constant;
      current_balance_param_.foot_pitch_torque_time_constant = current_update_gain*(desired_balance_param_.foot_pitch_torque_time_constant - previous_balance_param_.foot_pitch_torque_time_constant) + previous_balance_param_.foot_pitch_torque_time_constant;

      setBalanceParam(current_balance_param_);
    }
  }

  online_walking->current_right_fx_N_  = r_foot_fx_N_;
  online_walking->current_right_fy_N_  = r_foot_fy_N_;
  online_walking->current_right_fz_N_  = r_foot_fz_N_;
  online_walking->current_right_tx_Nm_ = r_foot_Tx_Nm_;
  online_walking->current_right_ty_Nm_ = r_foot_Ty_Nm_;
  online_walking->current_right_tz_Nm_ = r_foot_Tz_Nm_;

  online_walking->current_left_fx_N_  = l_foot_fx_N_;
  online_walking->current_left_fy_N_  = l_foot_fy_N_;
  online_walking->current_left_fz_N_  = l_foot_fz_N_;
  online_walking->current_left_tx_Nm_ = l_foot_Tx_Nm_;
  online_walking->current_left_ty_Nm_ = l_foot_Ty_Nm_;
  online_walking->current_left_tz_Nm_ = l_foot_Tz_Nm_;


  online_walking->process();

  publish_mutex_.lock();
  desired_matrix_g_to_cob_   = online_walking->mat_g_to_cob_;
  desired_matrix_g_to_rfoot_ = online_walking->mat_g_to_rfoot_;
  desired_matrix_g_to_lfoot_ = online_walking->mat_g_to_lfoot_;
  publish_mutex_.unlock();

  publishRobotPose();

  result_["r_leg_hip_y"]->goal_position_ = online_walking->out_angle_rad_[0];
  result_["r_leg_hip_r"]->goal_position_ = online_walking->out_angle_rad_[1];
  result_["r_leg_hip_p"]->goal_position_ = online_walking->out_angle_rad_[2];
  result_["r_leg_kn_p" ]->goal_position_ = online_walking->out_angle_rad_[3];
  result_["r_leg_an_p" ]->goal_position_ = online_walking->out_angle_rad_[4];
  result_["r_leg_an_r" ]->goal_position_ = online_walking->out_angle_rad_[5];

  result_["l_leg_hip_y"]->goal_position_ = online_walking->out_angle_rad_[6];
  result_["l_leg_hip_r"]->goal_position_ = online_walking->out_angle_rad_[7];
  result_["l_leg_hip_p"]->goal_position_ = online_walking->out_angle_rad_[8];
  result_["l_leg_kn_p" ]->goal_position_ = online_walking->out_angle_rad_[9];
  result_["l_leg_an_p" ]->goal_position_ = online_walking->out_angle_rad_[10];
  result_["l_leg_an_r" ]->goal_position_ = online_walking->out_angle_rad_[11];

  present_running = isRunning();
  if(previous_running_ != present_running)
  {
    if(present_running == true)
    {
      std::string status_msg = WalkingStatusMSG::WALKING_START_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      std::string status_msg = WalkingStatusMSG::WALKING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
  }
  previous_running_ = present_running;
}

void WalkingMotionModule::stop()
{
  return;
}

