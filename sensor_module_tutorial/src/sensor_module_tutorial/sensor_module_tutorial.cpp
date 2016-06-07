/*
 * sensor_module_tutorial.cpp
 *
 *  Created on: 2016. 4. 20.
 *      Author: zerom
 */

#include <stdio.h>
#include "sensor_module_tutorial/sensor_module_tutorial.h"

using namespace ROBOTIS;

SensorModuleTutorial::SensorModuleTutorial()
    : control_cycle_msec_(8)
{
    module_name     = "test_sensor_module"; // set unique module name

    result["test_sensor"] = 0.0;
}

SensorModuleTutorial::~SensorModuleTutorial()
{
    queue_thread_.join();
}

void SensorModuleTutorial::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_       = boost::thread(boost::bind(&SensorModuleTutorial::QueueThread, this));
}

void SensorModuleTutorial::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscriber */
    sub1_ = _ros_node.subscribe("/tutorial_topic", 10, &SensorModuleTutorial::TopicCallback, this);

    /* publisher */
    pub1_ = _ros_node.advertise<std_msgs::Int16>("/tutorial_publish", 1, true);

    ros::WallDuration duration(control_cycle_msec_/1000.0);
    while(_ros_node.ok())
      _callback_queue.callAvailable(duration);
}

void SensorModuleTutorial::TopicCallback(const std_msgs::Int16::ConstPtr &msg)
{
    std_msgs::Int16 _msg;
    _msg.data = msg->data;
    pub1_.publish(_msg);
}

void SensorModuleTutorial::Process(std::map<std::string, Dynamixel *> dxls)
{
    UINT16_T ext_port_data_1 = dxls["r_leg_an_p"]->dxl_state->bulk_read_table["external_port_data_1"];
    UINT16_T ext_port_data_2 = dxls["r_leg_an_p"]->dxl_state->bulk_read_table["external_port_data_2"];

    // ...

    result["test_sensor"] = 0.0;
}


