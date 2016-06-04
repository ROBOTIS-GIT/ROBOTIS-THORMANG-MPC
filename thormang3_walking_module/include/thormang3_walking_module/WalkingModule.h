/*
 * TestMotionModule.h
 *
 *  Created on: 2016. 1. 25.
 *      Author: zerom
 */

#ifndef WALKING_MODULE_H_
#define WALKING_MODULE_H_


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>


#include "PreviewControlWalking.h"
#include "WalkingModuleCommon.h"

#include "robotis_framework_common/MotionModule.h"


#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/WalkingStart.h"
#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"
//#include "thormang3_base_module_msgs/CalibrationWrench.h"


//#include "imu_3dm_gx4/FilterOutput.h"


namespace ROBOTIS
{

class WalkingMotionModule : public MotionModule, public Singleton<WalkingMotionModule>
{
private:
    int             control_cycle_msec_;
    boost::thread   queue_thread_;
    boost::mutex    publish_mutex_;

    Eigen::MatrixXd RX_PI_3x3, RZ_PI_3x3;
    Eigen::MatrixXd desired_matrix_g_to_cob_;
    Eigen::MatrixXd desired_matrix_g_to_rfoot_;
    Eigen::MatrixXd desired_matrix_g_to_lfoot_;


    void QueueThread();

    bool previous_enable, present_enable;
    bool previous_running, present_running;

    /* ROS Topic Publish Functions */
    int r_foot_ft_publish_checker_;
    int l_foot_ft_publish_checker_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher status_msg_pub_;

    thormang3_walking_module_msgs::RobotPose  robot_pose_msg_;
    bool	balance_update_with_loop_;
    double  balance_update_duration_;
    double  balance_update_sys_time_;
    Eigen::MatrixXd balance_update_polynomial_coeff_;
    thormang3_walking_module_msgs::BalanceParam previous_balance_param_;
    thormang3_walking_module_msgs::BalanceParam current_balance_param_;
    thormang3_walking_module_msgs::BalanceParam desired_balance_param_;

    void	PublishRobotPose(void);

    void	PublishStatusMsg(unsigned int type, std::string msg);

    /* ROS Topic Callback Functions */
//    void	IMUFilterOutputCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &msg);
    void	IMUDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);
    //void	FTSensorCalibrationDataCallback(const thormang3_base_module_msgs::CalibrationWrench::ConstPtr &msg);

    /* ROS Service Callback Functions */
    bool 	GetReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request  &req,
    											thormang3_walking_module_msgs::GetReferenceStepData::Response &res);
    bool 	AddStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request  &req,
    									thormang3_walking_module_msgs::AddStepDataArray::Response &res);
    bool	WalkingStartServiceCallback(thormang3_walking_module_msgs::WalkingStart::Request  &req,
    									thormang3_walking_module_msgs::WalkingStart::Response &res);
    bool	IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request  &req,
    									thormang3_walking_module_msgs::IsRunning::Response &res);
    bool	SetBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
    										thormang3_walking_module_msgs::SetBalanceParam::Response &res);
    bool	RemoveExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
    												thormang3_walking_module_msgs::RemoveExistingStepData::Response &res);

    int		StepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, StepData& des);
    int		StepDataToStepDataMsg(StepData& src, thormang3_walking_module_msgs::StepData& des);

    void	SetBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg);

public:
    WalkingMotionModule();
    virtual ~WalkingMotionModule();

    double gyro_x, gyro_y;
    double orientation_roll, orientation_pitch;
    double r_foot_fx_N,  r_foot_fy_N,  r_foot_fz_N;
    double r_foot_Tx_Nm, r_foot_Ty_Nm, r_foot_Tz_Nm;
    double l_foot_fx_N,  l_foot_fy_N,  l_foot_fz_N;
    double l_foot_Tx_Nm, l_foot_Ty_Nm, l_foot_Tz_Nm;

    void    Initialize(const int control_cycle_msec, Robot *robot);
    //void	ForceTorqueSensorInitialize();
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void	Stop();
    bool	IsRunning();
};

}


#endif /* WALKING_MODULE_H_ */
