/*
 * motion_module_tutorial.cpp
 *
 *  Created on: 2016. 2. 23.
 *      Author: zerom
 */

#include <stdio.h>
#include "motion_module_tutorial/motion_module_tutorial.h"

using namespace ROBOTIS;

MotionModuleTutorial::MotionModuleTutorial()
    : control_cycle_msec_(8)
{
    enable          = false;
    module_name     = "test_motion_module"; // set unique module name
    control_mode    = POSITION_CONTROL;

    result["r_sho_pitch"]   = new DynamixelState();
    result["r_sho_roll"]    = new DynamixelState();
    result["r_el"]          = new DynamixelState();
}

MotionModuleTutorial::~MotionModuleTutorial()
{
    queue_thread_.join();
}

void MotionModuleTutorial::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_       = boost::thread(boost::bind(&MotionModuleTutorial::QueueThread, this));
}

void MotionModuleTutorial::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscriber */
    sub1_ = _ros_node.subscribe("/tutorial_topic", 10, &MotionModuleTutorial::TopicCallback, this);

    /* publisher */
    pub1_ = _ros_node.advertise<std_msgs::Int16>("/tutorial_publish", 1, true);

    ros::WallDuration duration(control_cycle_msec_/1000.0);
    while(_ros_node.ok())
      _callback_queue.callAvailable(duration);
}

void MotionModuleTutorial::TopicCallback(const std_msgs::Int16::ConstPtr &msg)
{
    std_msgs::Int16 _msg;
    _msg.data = msg->data;
    pub1_.publish(_msg);
}

void MotionModuleTutorial::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
    if(enable == false)
        return;

    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        int32_t p_pos = dxls[state_iter->first]->dxl_state->present_position;
        int32_t g_pos = dxls[state_iter->first]->dxl_state->goal_position;
    }

    // ...

    result["r_sho_pitch"]->goal_position    = 0;
    result["r_sho_roll"]->goal_position     = 0;
    result["r_el"]->goal_position           = 0;
}

void MotionModuleTutorial::Stop()
{
	return;
}

bool MotionModuleTutorial::IsRunning()
{
	return false;
}



