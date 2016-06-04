/*
 * ThorManipulation.h
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#ifndef BASEMODULE_H_
#define BASEMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "robotis_framework_common/MotionModule.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_math/RobotisMath.h"
#include "thormang3_kinematics_dynamics/ThorMang3KinematicsDynamics.h"

#include "RobotisState.h"

namespace ROBOTIS
{

//using namespace ROBOTIS_BASE;

class BaseJointData
{

public:
    double position;
    double velocity;
    double effort;

    int p_gain;
    int i_gain;
    int d_gain;

};

class BaseJointState
{

public:
    BaseJointData curr_joint_state[ MAX_JOINT_ID + 1 ];
    BaseJointData goal_joint_state[ MAX_JOINT_ID + 1 ];
    BaseJointData fake_joint_state[ MAX_JOINT_ID + 1 ];

};

class BaseModule : public MotionModule, public Singleton<BaseModule>
{
private:
    int                 control_cycle_msec_;
    boost::thread       queue_thread_;
    boost::thread       tra_gene_tread_;

    // ros::Publisher      send_tra_pub_;
    ros::Publisher      status_msg_pub_;
    ros::Publisher		set_ctrl_module_pub_;

    std::map<std::string, int> joint_name_to_id;

    bool				has_goal_joints_;
    bool 				ini_pose_only_;


    void QueueThread();
    void SetCtrlModule(std::string module);

    void ParseIniPoseData(const std::string &path);

    void PublishStatusMsg(unsigned int type, std::string msg);

public:
    BaseModule();
    virtual ~BaseModule();

    /* ROS Topic Callback Functions */
    void    IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );

    /* ROS Calculation Functions */
    void    IniposeTraGeneProc();

    void 	PoseGenProc(Eigen::MatrixXd _joint_angle_pose);
    void 	PoseGenProc(std::map<std::string, double>& joint_angle_pose);

    /* ROS Framework Functions */
    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void	Stop();
    bool	IsRunning();

    /* Parameter */
    ROBOTIS_BASE::RobotisState *Robotis;
    BaseJointState *JointState;

};

}


#endif /* BASEMODULE_H_ */
