/*
 * ThorMang3FootForceTorqueSensorModule.h
 *
 *  Created on: Mar 22, 2016
 *      Author: jay
 */

#ifndef THORMANG3_FEET_FORCE_TORQUE_SENSOR_MODULE_H_
#define THORMANG3_FEET_FORCE_TORQUE_SENSOR_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <std_msgs/String.h>

#include <fstream>

#include "robotis_math/RobotisMath.h"

#include "thormang3_feet_ft_module_msgs/BothWrench.h"


#include "thormang3_kinematics_dynamics/ThorMang3KinematicsDynamics.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/SensorModule.h"

#include "ati_ft_sensor/ATIForceTorqueSensor.h"



namespace ROBOTIS
{

class ThorMang3FeetForceTorqueSensor : public SensorModule, public Singleton<ThorMang3FeetForceTorqueSensor>
{
private:
    int             control_cycle_msec_;
    boost::thread   queue_thread_;
    boost::mutex    publish_mutex_;

    ThorMang3KinematicsDynamics* thormang3_kd_;

	bool exist_r_leg_an_r_;
	bool exist_r_leg_an_p_;
	bool exist_l_leg_an_r_;
	bool exist_l_leg_an_p_;

    void QueueThread();


    ATIForceTorqueSensorTWE r_foot_ft_sensor_;
    ATIForceTorqueSensorTWE l_foot_ft_sensor_;

    Eigen::MatrixXd r_foot_ft_air_, l_foot_ft_air_;
    Eigen::MatrixXd r_foot_ft_gnd_, l_foot_ft_gnd_;

    double r_foot_ft_current_voltage_[6];
    double l_foot_ft_current_voltage_[6];

    double total_mass_;
    double r_foot_ft_scale_factor_, l_foot_ft_scale_factor_;


	ros::Publisher  thormang3_foot_ft_status_pub_;
	ros::Publisher  thormang3_foot_ft_both_ft_pub_;

	void FootForceTorqueSensorInitialize();
	void SaveFTCalibrationData(const std::string &path);

	void FTSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg);
    void PublishStatusMsg(unsigned int type, std::string msg);
    void PublishBothFTData(int type, Eigen::MatrixXd &ft_right, Eigen::MatrixXd &ft_left);

    bool	has_ft_air_;
    bool	has_ft_gnd_;
    int 	ft_command_;
    int		ft_period_;
    int		ft_get_count_;

    enum
	{
    	FT_NONE = 0,
    	FT_AIR = 1,
		FT_GND = 2,
		FT_CALC = 3,
	};

public:
    ThorMang3FeetForceTorqueSensor();
    ~ThorMang3FeetForceTorqueSensor();


	double r_foot_fx_raw_N,  r_foot_fy_raw_N,  r_foot_fz_raw_N;
	double r_foot_tx_raw_Nm, r_foot_ty_raw_Nm, r_foot_tz_raw_Nm;
	double l_foot_fx_raw_N,  l_foot_fy_raw_N,  l_foot_fz_raw_N;
	double l_foot_tx_raw_Nm, l_foot_ty_raw_Nm, l_foot_tz_raw_Nm;

	double r_foot_fx_scaled_N,  r_foot_fy_scaled_N,  r_foot_fz_scaled_N;
	double r_foot_tx_scaled_Nm, r_foot_ty_scaled_Nm, r_foot_tz_scaled_Nm;
	double l_foot_fx_scaled_N,  l_foot_fy_scaled_N,  l_foot_fz_scaled_N;
	double l_foot_tx_scaled_Nm, l_foot_ty_scaled_Nm, l_foot_tz_scaled_Nm;


    /* ROS Topic Callback Functions */
    //void    TopicCallback(const std_msgs::Int16::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, Sensor *> sensors);

    void	Stop();
    bool	IsRunning();


};


}


#endif /* THORMANG3_FEET_FORCE_TORQUE_SENSOR_MODULE_H_ */
