//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef ROS_CONTROL_MODULE_H_
#define ROS_CONTROL_MODULE_H_

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>

// robotis
#include <robotis_framework_common/motion_module.h>

// ros control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>



namespace thormang3
{
class RosControlModule
  : public robotis_framework::Singleton<RosControlModule>
  , public robotis_framework::MotionModule
  , public hardware_interface::RobotHW
{
public:
  RosControlModule();
  virtual ~RosControlModule();

  void onModuleEnable() override {}
  void onModuleDisable() override {}

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;

  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> sensors) override;

  void stop() override;
  bool isRunning() override;

private:
  void controllerManagerThread();
  void queueThread();

  int control_cycle_msec_;
  boost::thread controller_manager_thread_;
  boost::thread queue_thread_;
  ros::Time last_time_stamp_;

  bool reset_controllers_;

  boost::mutex ros_control_mutex_;
  
  /** ROS CONTROL PART */
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // IMU
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  hardware_interface::ImuSensorHandle::Data imu_data_;
  double imu_orientation_[4];
  double imu_angular_velocity_[3];
  double imu_linear_acceleration_[3];

  // FT-Sensors
  hardware_interface::ForceTorqueSensorInterface force_torque_sensor_interface_;
  std::map<std::string, double[3]> force_;
  std::map<std::string, double[3]> torque_;
  
  // joint interfaces
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  std::map<std::string, double> cmd_;
  std::map<std::string, double> pos_;
  std::map<std::string, double> vel_;
  std::map<std::string, double> eff_;
};
}

#endif

