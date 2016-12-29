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

#ifndef THORMANG3_IMU_SENSOR_MODULE_H_
#define THORMANG3_IMU_SENSOR_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <sensor_msgs/Imu.h>

#include <robotis_framework_common/sensor_module.h>



namespace thormang3
{
class ImuSensor : public robotis_framework::SensorModule, public robotis_framework::Singleton<ImuSensor>
{
public:
  ImuSensor();
  ~ImuSensor();

  /* ROS Topic Callback Functions */
  void imuSensorCallback(const sensor_msgs::Imu::ConstPtr msg, const std::string& tag);

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel* > dxls, std::map<std::string, robotis_framework::Sensor*> sensors) override;

  void stop();
  bool isRunning();

  bool gazebo_mode_;
  std::string gazebo_robot_name_;

private:
  void msgQueueThread();

  void publishStatusMsg(unsigned int type, std::string msg);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::mutex imu_sensor_mutex_;

  ros::Publisher  imu_status_pub_;
};
}


#endif /* THORMANG3_FEET_FORCE_TORQUE_SENSOR_MODULE_H_ */
