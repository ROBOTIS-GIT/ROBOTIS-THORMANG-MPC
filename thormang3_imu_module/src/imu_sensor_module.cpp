#include <thormang3_imu_module/imu_sensor_module.h>

#include <tf/tf.h>
#include <robotis_controller_msgs/StatusMsg.h>



using namespace thormang3;

ImuSensor::ImuSensor()
  : control_cycle_msec_(8)
  , gazebo_robot_name_("robotis")
  , gazebo_mode_(false)
{
  module_name_ = "thormang3_imu_sensor_module"; // set unique module name

  result_["imu_orientation_q_x_raw"] = 0.0;
  result_["imu_orientation_q_y_raw"] = 0.0;
  result_["imu_orientation_q_z_raw"] = 0.0;
  result_["imu_orientation_q_w_raw"] = 1.0;

  result_["imu_orientation_r_raw"] = 0.0;
  result_["imu_orientation_p_raw"] = 0.0;
  result_["imu_orientation_y_raw"] = 0.0;

  result_["imu_linear_acceleration_x_raw"] = 0.0;
  result_["imu_linear_acceleration_y_raw"] = 0.0;
  result_["imu_linear_acceleration_z_raw"] = 0.0;

  result_["imu_angular_velocity_x_raw"] = 0.0;
  result_["imu_angular_velocity_y_raw"] = 0.0;
  result_["imu_angular_velocity_z_raw"] = 0.0;

  result_["imu_orientation_q_x_filtered"] = 0.0;
  result_["imu_orientation_q_y_filtered"] = 0.0;
  result_["imu_orientation_q_z_filtered"] = 0.0;
  result_["imu_orientation_q_w_filtered"] = 1.0;

  result_["imu_orientation_r_filtered"] = 0.0;
  result_["imu_orientation_p_filtered"] = 0.0;
  result_["imu_orientation_y_filtered"] = 0.0;

  result_["imu_linear_acceleration_x_filtered"] = 0.0;
  result_["imu_linear_acceleration_y_filtered"] = 0.0;
  result_["imu_linear_acceleration_z_filtered"] = 0.0;

  result_["imu_angular_velocity_x_filtered"] = 0.0;
  result_["imu_angular_velocity_y_filtered"] = 0.0;
  result_["imu_angular_velocity_z_filtered"] = 0.0;
}

ImuSensor::~ImuSensor()
{
  queue_thread_.join();
}

void ImuSensor::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;

  queue_thread_       = boost::thread(boost::bind(&ImuSensor::msgQueueThread, this));
}

void ImuSensor::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type = type;
  status.module_name = "IMU";
  status.status_msg = msg;

  imu_status_pub_.publish(status);
}

void ImuSensor::imuSensorCallback(const sensor_msgs::Imu::ConstPtr msg, const std::string& tag)
{
  boost::mutex::scoped_lock lock(imu_sensor_mutex_);

  /// TODO: Handle covariance matrices

  result_["imu_orientation_q_x_" + tag] = msg->orientation.x;
  result_["imu_orientation_q_y_" + tag] = msg->orientation.y;
  result_["imu_orientation_q_z_" + tag] = msg->orientation.z;
  result_["imu_orientation_q_w_" + tag] = msg->orientation.w;

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);
  tf::Matrix3x3(q).getRPY(result_["imu_orientation_r_" + tag], result_["imu_orientation_p_" + tag], result_["imu_orientation_y_" + tag]);

  result_["imu_linear_acceleration_x_" + tag] = msg->linear_acceleration.x;
  result_["imu_linear_acceleration_y_" + tag] = msg->linear_acceleration.y;
  result_["imu_linear_acceleration_z_" + tag] = msg->linear_acceleration.z;

  result_["imu_angular_velocity_x_" + tag] = msg->angular_velocity.x;
  result_["imu_angular_velocity_y_" + tag] = msg->angular_velocity.y;
  result_["imu_angular_velocity_z_" + tag] = msg->angular_velocity.z;
}

void ImuSensor::msgQueueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  //ros::Subscriber ft_calib_command_sub	= _ros_node.subscribe("robotis/wrist_ft/ft_calib_command",	1, &ThorMang3ImuSensor::FTSensorCalibrationCommandCallback, this);
  ros::Subscriber imu_raw_sub = ros_node.subscribe<sensor_msgs::Imu>(gazebo_mode_ ? "/gazebo/" + gazebo_robot_name_ + "/sensor/imu" : "sensor/imu/raw",	1,
                                                    boost::bind(&ImuSensor::imuSensorCallback, this, _1, std::string("raw")));
  ros::Subscriber imu_filtered_sub = ros_node.subscribe<sensor_msgs::Imu>(gazebo_mode_ ? "/gazebo/" + gazebo_robot_name_ + "/sensor/imu" : "sensor/imu/filtered",	1,
                                                    boost::bind(&ImuSensor::imuSensorCallback, this, _1, std::string("filtered")));

  /* publisher */
  imu_status_pub_	= ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

  ros::WallDuration duration(control_cycle_msec_/1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void ImuSensor::process(std::map<std::string, robotis_framework::Dynamixel*> /*dxls*/, std::map<std::string, robotis_framework::Sensor*> /*sensors*/)
{
  // nothing to do
}
