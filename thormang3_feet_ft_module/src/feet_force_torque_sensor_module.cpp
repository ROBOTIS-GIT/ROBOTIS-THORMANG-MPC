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
 * feet_force_torque_sensor_module.h
 *
 *  Created on: 2016. 3. 22.
 *      Author: Jay Song
 */

#include "thormang3_feet_ft_module/feet_force_torque_sensor_module.h"

#define EXT_PORT_DATA_1 "external_port_data_1"
#define EXT_PORT_DATA_2 "external_port_data_2"
#define EXT_PORT_DATA_3 "external_port_data_3"
#define EXT_PORT_DATA_4 "external_port_data_4"

using namespace thormang3;

FeetForceTorqueSensor::FeetForceTorqueSensor()
: control_cycle_msec_(8),
  FT_NONE(0),
  FT_AIR(1),
  FT_GND(2),
  FT_CALC(3)
{
  module_name_ = "thormang3_foot_force_torque_sensor_module"; // set unique module name

  thormang3_kd_ = new KinematicsDynamics(WholeBody);

  r_foot_ft_air_.resize(6, 1); r_foot_ft_air_.fill(0.0);
  l_foot_ft_air_.resize(6, 1); l_foot_ft_air_.fill(0.0);

  r_foot_ft_gnd_.resize(6, 1); r_foot_ft_gnd_.fill(0.0);
  l_foot_ft_gnd_.resize(6, 1); l_foot_ft_gnd_.fill(0.0);

  r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = 1.0;
  total_mass_ = thormang3_kd_->calcTotalMass(0);

  for(int _idx = 0; _idx < 6; _idx++) {
    r_foot_ft_current_voltage_[_idx] = 3.3*0.5;
    l_foot_ft_current_voltage_[_idx] = 3.3*0.5;
  }

  r_foot_fx_raw_N_  = r_foot_fy_raw_N_  = r_foot_fz_raw_N_  = 0;
  r_foot_tx_raw_Nm_ = r_foot_ty_raw_Nm_ = r_foot_tz_raw_Nm_ = 0;
  l_foot_fx_raw_N_  = l_foot_fy_raw_N_  = l_foot_fz_raw_N_  = 0;
  l_foot_tx_raw_Nm_ = l_foot_ty_raw_Nm_ = l_foot_tz_raw_Nm_ = 0;

  r_foot_fx_scaled_N_  = r_foot_fy_scaled_N_  = r_foot_fz_scaled_N_  = 0;
  r_foot_tx_scaled_Nm_ = r_foot_ty_scaled_Nm_ = r_foot_tz_scaled_Nm_ = 0;
  l_foot_fx_scaled_N_  = l_foot_fy_scaled_N_  = l_foot_fz_scaled_N_  = 0;
  l_foot_tx_scaled_Nm_ = l_foot_ty_scaled_Nm_ = l_foot_tz_scaled_Nm_ = 0;


  result_["r_foot_fx_raw_N"]  = r_foot_fx_raw_N_;
  result_["r_foot_fy_raw_N"]  = r_foot_fy_raw_N_;
  result_["r_foot_fz_raw_N"]  = r_foot_fz_raw_N_;
  result_["r_foot_tx_raw_Nm"] = r_foot_tx_raw_Nm_;
  result_["r_foot_ty_raw_Nm"] = r_foot_ty_raw_Nm_;
  result_["r_foot_tz_raw_Nm"] = r_foot_tz_raw_Nm_;

  result_["l_foot_fx_raw_N"]  = l_foot_fx_raw_N_;
  result_["l_foot_fy_raw_N"]  = l_foot_fy_raw_N_;
  result_["l_foot_fz_raw_N"]  = l_foot_fz_raw_N_;
  result_["l_foot_tx_raw_Nm"] = l_foot_tx_raw_Nm_;
  result_["l_foot_ty_raw_Nm"] = l_foot_ty_raw_Nm_;
  result_["l_foot_tz_raw_Nm"] = l_foot_tz_raw_Nm_;


  result_["r_foot_fx_scaled_N"]  = r_foot_fx_scaled_N_;
  result_["r_foot_fy_scaled_N"]  = r_foot_fy_scaled_N_;
  result_["r_foot_fz_scaled_N"]  = r_foot_fz_scaled_N_;
  result_["r_foot_tx_scaled_Nm"] = r_foot_tx_scaled_Nm_;
  result_["r_foot_ty_scaled_Nm"] = r_foot_ty_scaled_Nm_;
  result_["r_foot_tz_scaled_Nm"] = r_foot_tz_scaled_Nm_;

  result_["l_foot_fx_scaled_N"]  = l_foot_fx_scaled_N_;
  result_["l_foot_fy_scaled_N"]  = l_foot_fy_scaled_N_;
  result_["l_foot_fz_scaled_N"]  = l_foot_fz_scaled_N_;
  result_["l_foot_tx_scaled_Nm"] = l_foot_tx_scaled_Nm_;
  result_["l_foot_ty_scaled_Nm"] = l_foot_ty_scaled_Nm_;
  result_["l_foot_tz_scaled_Nm"] = l_foot_tz_scaled_Nm_;


  exist_r_leg_an_r_ = false;
  exist_r_leg_an_p_ = false;
  exist_l_leg_an_r_ = false;
  exist_l_leg_an_p_ = false;


  has_ft_air_ = false;
  has_ft_gnd_ = false;
  ft_command_ = FT_NONE;
  ft_period_  = 2 * 1000/ control_cycle_msec_;

  ft_get_count_ = 0;
}

FeetForceTorqueSensor::~FeetForceTorqueSensor()
{
  queue_thread_.join();
}

void FeetForceTorqueSensor::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;

  ft_period_    = 2 * 1000/ control_cycle_msec_;
  ft_get_count_ = ft_period_;


  queue_thread_ = boost::thread(boost::bind(&FeetForceTorqueSensor::queueThread, this));

  initializeFeetForceTorqueSensor();
}

void FeetForceTorqueSensor::initializeFeetForceTorqueSensor()
{
  ros::NodeHandle ros_node;

  std::string foot_ft_data_path  = ros_node.param<std::string>("ft_data_path", "");
  std::string ft_calib_data_path = ros_node.param<std::string>("ft_calibration_data_path", "");

  r_foot_ft_sensor_.initialize(foot_ft_data_path, "ft_right_foot", "r_foot_ft_link" , "/robotis/sensor/ft_right_foot/raw", "/robotis/sensor/ft_right_foot/scaled");
  l_foot_ft_sensor_.initialize(foot_ft_data_path, "ft_left_foot",  "l_foot_ft_link",  "/robotis/sensor/ft_left_foot/raw",  "/robotis/sensor/ft_left_foot/scaled");


  YAML::Node doc;

  doc = YAML::LoadFile(ft_calib_data_path.c_str());

  std::vector<double> ft;
  ft = doc["ft_right_foot_air"].as<std::vector<double> >();
  r_foot_ft_air_ = Eigen::Map<Eigen::MatrixXd>(ft.data(), 6, 1);

  ft.clear();
  ft = doc["ft_right_foot_gnd"].as<std::vector<double> >();
  r_foot_ft_gnd_ = Eigen::Map<Eigen::MatrixXd>(ft.data(), 6, 1);

  ft.clear();
  ft = doc["ft_left_foot_air"].as<std::vector<double> >();
  l_foot_ft_air_ = Eigen::Map<Eigen::MatrixXd>(ft.data(), 6, 1);

  ft.clear();
  ft = doc["ft_left_foot_gnd"].as<std::vector<double> >();
  l_foot_ft_gnd_ = Eigen::Map<Eigen::MatrixXd>(ft.data(), 6, 1);

  double scale = ( r_foot_ft_gnd_.coeff(2, 0) + l_foot_ft_gnd_.coeff(2, 0) - r_foot_ft_air_.coeff(2, 0) - l_foot_ft_air_.coeff(2, 0) ) / (total_mass_ * GRAVITY_ACCELERATION);
  r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = scale;

  r_foot_ft_sensor_.setScaleParam(r_foot_ft_scale_factor_, r_foot_ft_air_);
  l_foot_ft_sensor_.setScaleParam(l_foot_ft_scale_factor_, l_foot_ft_air_);
}

void FeetForceTorqueSensor::saveFTCalibrationData(const std::string &path)
{
  if(has_ft_air_ == false || has_ft_gnd_ == false) return;

  YAML::Emitter out;

  out << YAML::BeginMap;

  // air - right
  std::vector<double> ft_calibration;
  for(int ix = 0; ix < 6; ix++)
    ft_calibration.push_back(r_foot_ft_air_.coeff(ix, 0));
  out << YAML::Key << "ft_right_foot_air" << YAML::Value << ft_calibration;

  // ground - right
  ft_calibration.clear();
  for(int ix = 0; ix < 6; ix++)
    ft_calibration.push_back(r_foot_ft_gnd_.coeff(ix, 0));
  out << YAML::Key << "ft_right_foot_gnd" << YAML::Value << ft_calibration;

  // air - left
  ft_calibration.clear();
  for(int ix = 0; ix < 6; ix++)
    ft_calibration.push_back(l_foot_ft_air_.coeff(ix, 0));
  out << YAML::Key << "ft_left_foot_air" << YAML::Value << ft_calibration;

  // ground - left
  ft_calibration.clear();
  for(int ix = 0; ix < 6; ix++)
    ft_calibration.push_back(l_foot_ft_gnd_.coeff(ix, 0));
  out << YAML::Key << "ft_left_foot_gnd" << YAML::Value << ft_calibration;

  out << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out.c_str();

  ROS_INFO("Save FT foot calibration data");
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Saved FT Calibration Data");
}

void  FeetForceTorqueSensor::ftSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  if( (ft_command_ == FT_NONE) && (ft_period_ == ft_get_count_) )
  {
    std::string command = msg->data;

    if(command == "ft_air")
    {
      ft_get_count_ = 0;
      ft_command_ = FT_AIR;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start measuring FT_AIR");
      has_ft_air_  = false;
      r_foot_ft_air_.fill(0);
      l_foot_ft_air_.fill(0);

    }
    else if(command == "ft_gnd")
    {
      ft_get_count_ = 0;
      ft_command_ = FT_GND;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start measuring FT_GND");
      has_ft_gnd_  = false;
      r_foot_ft_gnd_.fill(0);
      l_foot_ft_gnd_.fill(0);
    }
    else if(command == "ft_send")
    {
      if(has_ft_air_ && has_ft_gnd_)
      {
        double scale = 1.0;
        scale = ( r_foot_ft_gnd_.coeff(2, 0) + l_foot_ft_gnd_.coeff(2, 0) - r_foot_ft_air_.coeff(2, 0) - l_foot_ft_air_.coeff(2, 0) ) / (total_mass_ * GRAVITY_ACCELERATION);
        r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = scale;

        ROS_INFO_STREAM("r_foot_ft_scale_factor_ : " << r_foot_ft_scale_factor_);
        ROS_INFO_STREAM("l_foot_ft_scale_factor_ : " << l_foot_ft_scale_factor_);
        ROS_INFO_STREAM("r_foot_ft_air_ : " << r_foot_ft_air_.transpose());
        ROS_INFO_STREAM("l_foot_ft_air_ : " << l_foot_ft_air_.transpose());
        r_foot_ft_sensor_.setScaleParam(r_foot_ft_scale_factor_, r_foot_ft_air_);
        l_foot_ft_sensor_.setScaleParam(l_foot_ft_scale_factor_, l_foot_ft_air_);

        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Applied FT Calibration");
      }
      else
      {
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "There is no value for calibration");
      }
    }
    else if(command == "ft_save")
    {
      ros::NodeHandle ros_node;
      std::string ft_calib_data_path = ros_node.param<std::string>("ft_calibration_data_path", "");
      saveFTCalibrationData(ft_calib_data_path);
    }
  }
  else
    ROS_INFO("previous task is alive");
}

void FeetForceTorqueSensor::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "FeetFT";
  status_msg.status_msg = msg;

  thormang3_foot_ft_status_pub_.publish(status_msg);
}

void FeetForceTorqueSensor::publishBothFTData(int type, Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left)
{
  thormang3_feet_ft_module_msgs::BothWrench both_wrench_msg;

  if(type == FT_AIR)
    both_wrench_msg.name = "ft_air";
  else if(type == FT_GND)
    both_wrench_msg.name = "ft_gnd";
  else
    return;

  both_wrench_msg.right.force.x  = ft_right.coeff(0,0);
  both_wrench_msg.right.force.y  = ft_right.coeff(1,0);
  both_wrench_msg.right.force.z  = ft_right.coeff(2,0);
  both_wrench_msg.right.torque.x = ft_right.coeff(3,0);
  both_wrench_msg.right.torque.y = ft_right.coeff(4,0);
  both_wrench_msg.right.torque.z = ft_right.coeff(5,0);

  both_wrench_msg.left.force.x  = ft_left.coeff(0,0);
  both_wrench_msg.left.force.y  = ft_left.coeff(1,0);
  both_wrench_msg.left.force.z  = ft_left.coeff(2,0);
  both_wrench_msg.left.torque.x = ft_left.coeff(3,0);
  both_wrench_msg.left.torque.y = ft_left.coeff(4,0);
  both_wrench_msg.left.torque.z = ft_left.coeff(5,0);

  thormang3_foot_ft_both_ft_pub_.publish(both_wrench_msg);
}

void FeetForceTorqueSensor::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  ros::Subscriber ft_calib_command_sub = ros_node.subscribe("/robotis/feet_ft/ft_calib_command", 1, &FeetForceTorqueSensor::ftSensorCalibrationCommandCallback, this);

  /* publisher */
  thormang3_foot_ft_status_pub_  = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  thormang3_foot_ft_both_ft_pub_ = ros_node.advertise<thormang3_feet_ft_module_msgs::BothWrench>("/robotis/feet_ft/both_ft_value", 1);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void FeetForceTorqueSensor::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors)
{
  exist_r_leg_an_r_ = false;
  exist_r_leg_an_p_ = false;
  exist_l_leg_an_r_ = false;
  exist_l_leg_an_p_ = false;

  std::map<std::string, robotis_framework::Dynamixel*>::iterator dxls_it = dxls.find("r_leg_an_r");


  if(dxls_it != dxls.end())
  {
    r_foot_ft_current_voltage_[0] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
    r_foot_ft_current_voltage_[1] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
    r_foot_ft_current_voltage_[2] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_3])*3.3/4095.0;
    r_foot_ft_current_voltage_[3] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_4])*3.3/4095.0;
    exist_r_leg_an_r_ = true;
  }
  else
    return;

  dxls_it = dxls.find("r_leg_an_p");
  if(dxls_it != dxls.end())
  {
    r_foot_ft_current_voltage_[4] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
    r_foot_ft_current_voltage_[5] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
    exist_r_leg_an_p_ = true;
  }
  else
    return;

  dxls_it = dxls.find("l_leg_an_r");
  if(dxls_it != dxls.end())
  {
    l_foot_ft_current_voltage_[0] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
    l_foot_ft_current_voltage_[1] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
    l_foot_ft_current_voltage_[2] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_3])*3.3/4095.0;
    l_foot_ft_current_voltage_[3] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_4])*3.3/4095.0;
    exist_l_leg_an_r_ = true;
  }
  else
    return;

  dxls_it = dxls.find("l_leg_an_p");
  if(dxls_it != dxls.end())
  {
    l_foot_ft_current_voltage_[4] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_1])*3.3/4095.0;
    l_foot_ft_current_voltage_[5] = (dxls_it->second->dxl_state_->bulk_read_table_[EXT_PORT_DATA_2])*3.3/4095.0;
    exist_l_leg_an_p_ = true;
  }
  else
    return;



  if( exist_r_leg_an_r_ && exist_r_leg_an_p_) {

    r_foot_ft_sensor_.setCurrentVoltageOutputPublish( r_foot_ft_current_voltage_[0],
        r_foot_ft_current_voltage_[1],
        r_foot_ft_current_voltage_[2],
        r_foot_ft_current_voltage_[3],
        r_foot_ft_current_voltage_[4],
        r_foot_ft_current_voltage_[5]);

    r_foot_ft_sensor_.getCurrentForceTorqueRaw(&r_foot_fx_raw_N_,  &r_foot_fy_raw_N_,  &r_foot_fz_raw_N_,
        &r_foot_tx_raw_Nm_, &r_foot_ty_raw_Nm_, &r_foot_tz_raw_Nm_);
    r_foot_ft_sensor_.getCurrentForceTorqueScaled(&r_foot_fx_scaled_N_,  &r_foot_fy_scaled_N_,  &r_foot_fz_scaled_N_,
        &r_foot_tx_scaled_Nm_, &r_foot_ty_scaled_Nm_, &r_foot_tz_scaled_Nm_);

    result_["r_foot_fx_raw_N"]  = r_foot_fx_raw_N_;
    result_["r_foot_fy_raw_N"]  = r_foot_fy_raw_N_;
    result_["r_foot_fz_raw_N"]  = r_foot_fz_raw_N_;
    result_["r_foot_tx_raw_Nm"] = r_foot_tx_raw_Nm_;
    result_["r_foot_ty_raw_Nm"] = r_foot_ty_raw_Nm_;
    result_["r_foot_tz_raw_Nm"] = r_foot_tz_raw_Nm_;


    result_["r_foot_fx_scaled_N"]  = r_foot_fx_scaled_N_;
    result_["r_foot_fy_scaled_N"]  = r_foot_fy_scaled_N_;
    result_["r_foot_fz_scaled_N"]  = r_foot_fz_scaled_N_;
    result_["r_foot_tx_scaled_Nm"] = r_foot_tx_scaled_Nm_;
    result_["r_foot_ty_scaled_Nm"] = r_foot_ty_scaled_Nm_;
    result_["r_foot_tz_scaled_Nm"] = r_foot_tz_scaled_Nm_;

  }

  if( exist_l_leg_an_r_ && exist_l_leg_an_p_) {
    l_foot_ft_sensor_.setCurrentVoltageOutputPublish(l_foot_ft_current_voltage_[0],
        l_foot_ft_current_voltage_[1],
        l_foot_ft_current_voltage_[2],
        l_foot_ft_current_voltage_[3],
        l_foot_ft_current_voltage_[4],
        l_foot_ft_current_voltage_[5]);

    l_foot_ft_sensor_.getCurrentForceTorqueRaw(&l_foot_fx_raw_N_,  &l_foot_fy_raw_N_,  &l_foot_fz_raw_N_,
        &l_foot_tx_raw_Nm_, &l_foot_ty_raw_Nm_, &l_foot_tz_raw_Nm_);
    l_foot_ft_sensor_.getCurrentForceTorqueScaled(&l_foot_fx_scaled_N_,  &l_foot_fy_scaled_N_,  &l_foot_fz_scaled_N_,
        &l_foot_tx_scaled_Nm_, &l_foot_ty_scaled_Nm_, &l_foot_tz_scaled_Nm_);


    result_["l_foot_fx_raw_N"]  = l_foot_fx_raw_N_;
    result_["l_foot_fy_raw_N"]  = l_foot_fy_raw_N_;
    result_["l_foot_fz_raw_N"]  = l_foot_fz_raw_N_;
    result_["l_foot_tx_raw_Nm"] = l_foot_tx_raw_Nm_;
    result_["l_foot_ty_raw_Nm"] = l_foot_ty_raw_Nm_;
    result_["l_foot_tz_raw_Nm"] = l_foot_tz_raw_Nm_;

    result_["l_foot_fx_scaled_N"]  = l_foot_fx_scaled_N_;
    result_["l_foot_fy_scaled_N"]  = l_foot_fy_scaled_N_;
    result_["l_foot_fz_scaled_N"]  = l_foot_fz_scaled_N_;
    result_["l_foot_tx_scaled_Nm"] = l_foot_tx_scaled_Nm_;
    result_["l_foot_ty_scaled_Nm"] = l_foot_ty_scaled_Nm_;
    result_["l_foot_tz_scaled_Nm"] = l_foot_tz_scaled_Nm_;
  }


  if(ft_command_ == FT_NONE )
    return;
  else if(ft_command_ == FT_AIR)
  {
    ft_get_count_++;

    r_foot_ft_air_.coeffRef(0, 0) += r_foot_fx_raw_N_;
    r_foot_ft_air_.coeffRef(1, 0) += r_foot_fy_raw_N_;
    r_foot_ft_air_.coeffRef(2, 0) += r_foot_fz_raw_N_;
    r_foot_ft_air_.coeffRef(3, 0) += r_foot_tx_raw_Nm_;
    r_foot_ft_air_.coeffRef(4, 0) += r_foot_ty_raw_Nm_;
    r_foot_ft_air_.coeffRef(5, 0) += r_foot_tz_raw_Nm_;

    l_foot_ft_air_.coeffRef(0, 0) += l_foot_fx_raw_N_;
    l_foot_ft_air_.coeffRef(1, 0) += l_foot_fy_raw_N_;
    l_foot_ft_air_.coeffRef(2, 0) += l_foot_fz_raw_N_;
    l_foot_ft_air_.coeffRef(3, 0) += l_foot_tx_raw_Nm_;
    l_foot_ft_air_.coeffRef(4, 0) += l_foot_ty_raw_Nm_;
    l_foot_ft_air_.coeffRef(5, 0) += l_foot_tz_raw_Nm_;

    if(ft_get_count_ == ft_period_)
    {
      r_foot_ft_air_  = r_foot_ft_air_ / (double)ft_period_;
      l_foot_ft_air_  = l_foot_ft_air_ / (double)ft_period_;

      has_ft_air_ = true;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish measuring FT_AIR");
      publishBothFTData(FT_AIR, r_foot_ft_air_, l_foot_ft_air_);
      ft_command_ = FT_NONE;
    }
  }
  else if(ft_command_ == FT_GND)
  {
    ft_get_count_++;

    r_foot_ft_gnd_.coeffRef(0, 0) += r_foot_fx_raw_N_;
    r_foot_ft_gnd_.coeffRef(1, 0) += r_foot_fy_raw_N_;
    r_foot_ft_gnd_.coeffRef(2, 0) += r_foot_fz_raw_N_;
    r_foot_ft_gnd_.coeffRef(3, 0) += r_foot_tx_raw_Nm_;
    r_foot_ft_gnd_.coeffRef(4, 0) += r_foot_ty_raw_Nm_;
    r_foot_ft_gnd_.coeffRef(5, 0) += r_foot_tz_raw_Nm_;

    l_foot_ft_gnd_.coeffRef(0, 0) += l_foot_fx_raw_N_;
    l_foot_ft_gnd_.coeffRef(1, 0) += l_foot_fy_raw_N_;
    l_foot_ft_gnd_.coeffRef(2, 0) += l_foot_fz_raw_N_;
    l_foot_ft_gnd_.coeffRef(3, 0) += l_foot_tx_raw_Nm_;
    l_foot_ft_gnd_.coeffRef(4, 0) += l_foot_ty_raw_Nm_;
    l_foot_ft_gnd_.coeffRef(5, 0) += l_foot_tz_raw_Nm_;

    if(ft_get_count_ == ft_period_)
    {
      r_foot_ft_gnd_  = r_foot_ft_gnd_ / (double)ft_period_;
      l_foot_ft_gnd_  = l_foot_ft_gnd_ / (double)ft_period_;

      has_ft_gnd_ = true;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish measuring FT_GND");
      publishBothFTData(FT_GND, r_foot_ft_gnd_, l_foot_ft_gnd_);

      ft_command_ = FT_NONE;
    }
  }
  else
  {
    return;
  }

}
