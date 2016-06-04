/*
 * motion_module_tutorial.h
 *
 *  Created on: 2016. 2. 23.
 *      Author: zerom
 */

#ifndef ROBOTIS_THORMANG_MPC_MOTION_MODULE_TUTORIAL_INCLUDE_MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_
#define ROBOTIS_THORMANG_MPC_MOTION_MODULE_TUTORIAL_INCLUDE_MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_



#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/MotionModule.h"

namespace ROBOTIS
{

class MotionModuleTutorial : public MotionModule, public Singleton<MotionModuleTutorial>
{
private:
    int             control_cycle_msec_;
    boost::thread   queue_thread_;

    /* sample subscriber & publisher */
    ros::Subscriber sub1_;
    ros::Publisher  pub1_;

    void QueueThread();

public:
    MotionModuleTutorial();
    virtual ~MotionModuleTutorial();

    /* ROS Topic Callback Functions */
    void    TopicCallback(const std_msgs::Int16::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void	Stop();
    bool	IsRunning();
};

}



#endif /* ROBOTIS_THORMANG_MPC_MOTION_MODULE_TUTORIAL_INCLUDE_MOTION_MODULE_TUTORIAL_MOTION_MODULE_TUTORIAL_H_ */
