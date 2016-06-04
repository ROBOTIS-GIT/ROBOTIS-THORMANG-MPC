/*
 * ThorManipulation.h
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#ifndef MANIPULATIONMODULE_H_
#define MANIPULATIONMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_framework_common/MotionModule.h"

#include "robotis_math/RobotisMath.h"
#include "thormang3_kinematics_dynamics/ThorMang3KinematicsDynamics.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "RobotisState.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"

#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

namespace ROBOTIS
{

class ManipulationJointData
{

public:
    double position;
    double velocity;
    double effort;

    int p_gain;
    int i_gain;
    int d_gain;

};

class ManipulationJointState
{

public:
    ManipulationJointData curr_joint_state[ MAX_JOINT_ID + 1 ];
    ManipulationJointData goal_joint_state[ MAX_JOINT_ID + 1 ];
    ManipulationJointData fake_joint_state[ MAX_JOINT_ID + 1 ];

};

class ManipulationModule : public MotionModule, public Singleton<ManipulationModule>
{
private:
    int                 control_cycle_msec_;
    boost::thread       queue_thread_;
    boost::thread*       tra_gene_tread_;

    ros::Publisher      status_msg_pub_;

    std::map<std::string, int> joint_name_to_id;

    void QueueThread();

    void parseData( const std::string &path );
    void parseIniPoseData( const std::string &path );

public:
    ManipulationModule();
    virtual ~ManipulationModule();

    /* ROS Topic Callback Functions */
    void    IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );
    void    JointPoseMsgCallback( const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg );
    void    KinematicsPoseMsgCallback( const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg );

    bool    GetJointPoseCallback( thormang3_manipulation_module_msgs::GetJointPose::Request &req , thormang3_manipulation_module_msgs::GetJointPose::Response &res );
    bool    GetKinematicsPoseCallback( thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req , thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res );

    /* ROS Calculation Functions */
    void    IniposeTraGeneProc( );
    void    JointTraGeneProc( );
    void    TaskTraGeneProc( );

    /* ROS Framework Functions */
    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);
    void	Stop();
    bool	IsRunning();

    void    PublishStatusMsg(unsigned int type, std::string msg);

    /* Parameter */
    ThorMang3KinematicsDynamics *Humanoid;
    ROBOTIS_MANIPULATION::RobotisState *Robotis;
    ManipulationJointState *JointState;
};

}


#endif /* MANIPULATIONMODULE_H_ */
