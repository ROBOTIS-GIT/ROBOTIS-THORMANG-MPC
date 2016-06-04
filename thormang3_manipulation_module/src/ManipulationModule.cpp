/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "thormang3_manipulation_module/ManipulationModule.h"

using namespace ROBOTIS;

ManipulationModule *ManipulationModule::unique_instance_ = new ManipulationModule();

ManipulationModule::ManipulationModule()
    : control_cycle_msec_(0)
{
    enable          = false;
    module_name     = "manipulation_module";
    control_mode    = POSITION_CONTROL;

    /* arm */
    result["r_arm_sh_p1"]   = new DynamixelState();
    result["l_arm_sh_p1"]   = new DynamixelState();
    result["r_arm_sh_r"]    = new DynamixelState();
    result["l_arm_sh_r"]    = new DynamixelState();
    result["r_arm_sh_p2"]   = new DynamixelState();
    result["l_arm_sh_p2"]   = new DynamixelState();
    result["r_arm_el_y"]    = new DynamixelState();
    result["l_arm_el_y"]    = new DynamixelState();
    result["r_arm_wr_r"]    = new DynamixelState();
    result["l_arm_wr_r"]    = new DynamixelState();
    result["r_arm_wr_y"]    = new DynamixelState();
    result["l_arm_wr_y"]    = new DynamixelState();
    result["r_arm_wr_p"]    = new DynamixelState();
    result["l_arm_wr_p"]    = new DynamixelState();
    result["torso_y"]       = new DynamixelState();
    result["r_arm_grip"]    = new DynamixelState();
    result["l_arm_grip"]    = new DynamixelState();

    /* arm */
    joint_name_to_id["r_arm_sh_p1"] = 1;
    joint_name_to_id["l_arm_sh_p1"] = 2;
    joint_name_to_id["r_arm_sh_r"]  = 3;
    joint_name_to_id["l_arm_sh_r"]  = 4;
    joint_name_to_id["r_arm_sh_p2"] = 5;
    joint_name_to_id["l_arm_sh_p2"] = 6;
    joint_name_to_id["r_arm_el_y"]  = 7;
    joint_name_to_id["l_arm_el_y"]  = 8;
    joint_name_to_id["r_arm_wr_r"]  = 9;
    joint_name_to_id["l_arm_wr_r"]  = 10;
    joint_name_to_id["r_arm_wr_y"]  = 11;
    joint_name_to_id["l_arm_wr_y"]  = 12;
    joint_name_to_id["r_arm_wr_p"]  = 13;
    joint_name_to_id["l_arm_wr_p"]  = 14;
    joint_name_to_id["torso_y"]     = 27;
    joint_name_to_id["r_arm_grip"]  = 31;
    joint_name_to_id["l_arm_grip"]  = 30;

    /* etc */
    joint_name_to_id["r_arm_end"] = 35;
    joint_name_to_id["l_arm_end"] = 34;

    /* parameter */
    Humanoid = new ThorMang3KinematicsDynamics( WHOLE_BODY );
    Robotis = new ROBOTIS_MANIPULATION::RobotisState();
    JointState = new ManipulationJointState();
}

ManipulationModule::~ManipulationModule()
{
    queue_thread_.join();
}

void ManipulationModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&ManipulationModule::QueueThread, this));

    ros::NodeHandle _ros_node;

    /* publish topics */

    // for gui
    status_msg_pub_         = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    
    std::string _path = ros::package::getPath("thormang3_manipulation_module") + "/config/ik_weight.yaml";
    parseData( _path );
}

void ManipulationModule::parseData( const std::string &path )
{
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return;
    }

    YAML::Node _ik_weight_node = doc["weight_value"];
    for(YAML::iterator _it = _ik_weight_node.begin(); _it != _ik_weight_node.end() ; ++_it)
    {
        int _id = _it->first.as<int>();
        double _value = _it->second.as<double>();

        Robotis->ik_weight.coeffRef( _id , 0 ) = _value;
    }
}

void ManipulationModule::parseIniPoseData( const std::string &path )
{
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    // parse movement time
    double _mov_time;
    _mov_time = doc["mov_time"].as< double >();

    Robotis->mov_time = _mov_time;

    // parse target pose
    YAML::Node _tar_pose_node = doc["tar_pose"];
    for(YAML::iterator _it = _tar_pose_node.begin() ; _it != _tar_pose_node.end() ; ++_it)
    {
        int _id;
        double _value;

        _id = _it->first.as<int>();
        _value = _it->second.as<double>();

        Robotis->joint_ini_pose.coeffRef( _id , 0 ) = _value * deg2rad;
    }

    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;
    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );
}

void ManipulationModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscribe topics */
    ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/manipulation/ini_pose_msg", 5, &ManipulationModule::IniPoseMsgCallback, this);
    ros::Subscriber joint_pose_msg_sub = _ros_node.subscribe("/robotis/manipulation/joint_pose_msg", 5, &ManipulationModule::JointPoseMsgCallback, this);
    ros::Subscriber kinematics_pose_msg_sub = _ros_node.subscribe("/robotis/manipulation/kinematics_pose_msg", 5, &ManipulationModule::KinematicsPoseMsgCallback, this);

    /* service */
    ros::ServiceServer get_joint_pose_server = _ros_node.advertiseService("/robotis/manipulation/get_joint_pose", &ManipulationModule::GetJointPoseCallback, this);
    ros::ServiceServer get_kinematics_pose_server = _ros_node.advertiseService("/robotis/manipulation/get_kinematics_pose", &ManipulationModule::GetKinematicsPoseCallback, this);

    ros::WallDuration duration(control_cycle_msec_/1000.0);
    while(_ros_node.ok())
      _callback_queue.callAvailable(duration);
}

void ManipulationModule::IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg )
{
    if(enable == false)
        return;

    if ( Robotis->is_moving == false )
    {
        if ( msg->data == "ini_pose")
        {
            // parse initial pose
            std::string _ini_pose_path = ros::package::getPath("thormang3_manipulation_module") + "/config/ini_pose.yaml";
            parseIniPoseData( _ini_pose_path );

            tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::IniposeTraGeneProc, this));
            delete tra_gene_tread_;
        }
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

bool ManipulationModule::GetJointPoseCallback( thormang3_manipulation_module_msgs::GetJointPose::Request &req , thormang3_manipulation_module_msgs::GetJointPose::Response &res )
{
    if(enable == false)
        return false;

    for ( int _name_index = 1; _name_index <= MAX_JOINT_ID; _name_index++ )
    {
        if( Humanoid->thormang3_link_data[ _name_index ]->name == req.joint_name )
        {
            res.joint_value = JointState->goal_joint_state[ _name_index ].position ;
            return true;
        }
    }

    return false;
}

bool ManipulationModule::GetKinematicsPoseCallback( thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req , thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res )
{
    if(enable == false)
        return false;

    int _end_index;

    if ( req.group_name == "left_arm" )
        _end_index = id_l_arm_end;
    else if ( req.group_name == "right_arm")
        _end_index = id_r_arm_end;
    else if ( req.group_name == "left_arm_with_torso" )
        _end_index = id_l_arm_end;
    else if ( req.group_name == "right_arm_with_torso" )
        _end_index = id_r_arm_end;
    else
        return false;

    res.group_pose.position.x = Humanoid->thormang3_link_data[ _end_index ]->position.coeff(0,0);
    res.group_pose.position.y = Humanoid->thormang3_link_data[ _end_index ]->position.coeff(1,0);
    res.group_pose.position.z = Humanoid->thormang3_link_data[ _end_index ]->position.coeff(2,0);

    Eigen::Quaterniond _quaternion = rotation2quaternion( Humanoid->thormang3_link_data[ _end_index ]->orientation );

    res.group_pose.orientation.w = _quaternion.w();
    res.group_pose.orientation.x = _quaternion.x();
    res.group_pose.orientation.y = _quaternion.y();
    res.group_pose.orientation.z = _quaternion.z();

    return  true;
}

void ManipulationModule::KinematicsPoseMsgCallback( const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg )
{
    if(enable == false)
        return;

    Robotis->goal_kinematics_pose_msg = *msg;

    if ( Robotis->goal_kinematics_pose_msg.name == "left_arm" )
    {
        Robotis->ik_id_start = id_l_arm_start;
        Robotis->ik_id_end = id_l_arm_end;
    }
    else if ( Robotis->goal_kinematics_pose_msg.name == "right_arm" )
    {
        Robotis->ik_id_start = id_r_arm_start;
        Robotis->ik_id_end = id_r_arm_end;
    }
    else if ( Robotis->goal_kinematics_pose_msg.name == "left_arm_with_torso" )
    {
        Robotis->ik_id_start = id_torso;
        Robotis->ik_id_end = id_l_arm_end;
    }
    else if ( Robotis->goal_kinematics_pose_msg.name == "right_arm_with_torso" )
    {
        Robotis->ik_id_start = id_torso;
        Robotis->ik_id_end = id_r_arm_end;
    }

    if ( Robotis->is_moving == false )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::TaskTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void ManipulationModule::JointPoseMsgCallback( const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg )
{
    if(enable == false)
        return;

    Robotis->goal_joint_pose_msg = *msg;

    if ( Robotis->is_moving == false )
    {
        tra_gene_tread_ = new boost::thread(boost::bind(&ManipulationModule::JointTraGeneProc, this));
        delete tra_gene_tread_;
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void ManipulationModule::IniposeTraGeneProc()
{
    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_ini_pose.coeff( id , 0 );

        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                tar_value , 0.0 , 0.0 ,
                                                Robotis->smp_time , Robotis->mov_time );

        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->cnt = 0;
    Robotis->is_moving = true;

    ROS_INFO("[start] send trajectory");
}

void ManipulationModule::JointTraGeneProc()
{
    if ( Robotis->goal_joint_pose_msg.time <= 0.0 )
    {
        /* set movement time */
        double _tol = 10 * deg2rad; // rad per sec
        double _mov_time = 2.0;

        int _ctrl_id = joint_name_to_id[ Robotis->goal_joint_pose_msg.name ];

        double _ini_value = JointState->goal_joint_state[ _ctrl_id ].position;
        double _tar_value = Robotis->goal_joint_pose_msg.value;
        double _diff = fabs ( _tar_value - _ini_value );

        Robotis->mov_time =  _diff / _tol;
        int _all_time_steps = int( floor( ( Robotis->mov_time / Robotis->smp_time ) + 1.0 ) );
        Robotis->mov_time = double ( _all_time_steps - 1 ) * Robotis->smp_time;

        if ( Robotis->mov_time < _mov_time )
            Robotis->mov_time = _mov_time;
    }
    else
        Robotis->mov_time = Robotis->goal_joint_pose_msg.time;


    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

    /* calculate joint trajectory */
    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = JointState->goal_joint_state[ id ].position;

        if( Humanoid->thormang3_link_data[ id ]->name == Robotis->goal_joint_pose_msg.name )
            tar_value = Robotis->goal_joint_pose_msg.value;

        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                tar_value , 0.0 , 0.0 ,
                                                Robotis->smp_time , Robotis->mov_time );

        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->cnt = 0;
    Robotis->is_moving = true;

    ROS_INFO("[start] send trajectory");
}

void ManipulationModule::TaskTraGeneProc()
{
    if ( Robotis->goal_joint_pose_msg.time <= 0.0 )
    {
        /* set movement time */
        double _tol = 0.1; // m per sec
        double _mov_time = 2.0;

        double _diff = sqrt( pow( Humanoid->thormang3_link_data[ Robotis->ik_id_end ]->position.coeff( 0 , 0 ) - Robotis->goal_kinematics_pose_msg.pose.position.x , 2  ) +
                             pow( Humanoid->thormang3_link_data[ Robotis->ik_id_end ]->position.coeff( 1 , 0 ) - Robotis->goal_kinematics_pose_msg.pose.position.y , 2  ) +
                             pow( Humanoid->thormang3_link_data[ Robotis->ik_id_end ]->position.coeff( 2 , 0 ) - Robotis->goal_kinematics_pose_msg.pose.position.z , 2  ) );

        Robotis->mov_time = _diff / _tol;
        int _all_time_steps = int( floor( ( Robotis->mov_time / Robotis->smp_time ) + 1.0 ) );
        Robotis->mov_time = double ( _all_time_steps - 1 ) * Robotis->smp_time;

        if ( Robotis->mov_time < _mov_time )
            Robotis->mov_time = _mov_time;
    }
    else
        Robotis->mov_time = Robotis->goal_joint_pose_msg.time;


    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_task_tra.resize( Robotis->all_time_steps , 3 );

     /* calculate trajectory */
    for ( int dim = 0; dim < 3; dim++ )
    {
        double ini_value = Humanoid->thormang3_link_data[ Robotis->ik_id_end ]->position.coeff( dim , 0 );
        double tar_value;
        if ( dim == 0 )
            tar_value = Robotis->goal_kinematics_pose_msg.pose.position.x ;
        else if ( dim == 1 )
            tar_value = Robotis->goal_kinematics_pose_msg.pose.position.y ;
        else if ( dim == 2 )
            tar_value = Robotis->goal_kinematics_pose_msg.pose.position.z ;

        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                                                tar_value , 0.0 , 0.0 ,
                                                Robotis->smp_time , Robotis->mov_time );

        Robotis->calc_task_tra.block( 0 , dim , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->cnt = 0;
    Robotis->is_moving = true;
    Robotis->ik_solve = true;

    ROS_INFO("[start] send trajectory");
}

void ManipulationModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
    if(enable == false)
        return;

    /*----- write curr position -----*/

    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
        if(_dxl_it != dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

        double _joint_curr_position = _dxl->dxl_state->present_position;
        double _joint_goal_position = _dxl->dxl_state->goal_position;

        JointState->curr_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_curr_position;
        JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_goal_position;
    }

     /*----- forward kinematics -----*/

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
        Humanoid->thormang3_link_data[ id ]->joint_angle = JointState->goal_joint_state[ id ].position;

    Humanoid->ForwardKinematics( 0 );

    /* ----- send trajectory ----- */

    if ( Robotis->is_moving == true )
    {
        if ( Robotis->cnt == 0 )
        {
            PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

            Robotis->ik_start_rotation = Humanoid->thormang3_link_data[ Robotis->ik_id_end ]->orientation;
        }

        if ( Robotis->ik_solve == true )
        {
            /* ----- inverse kinematics ----- */
            Robotis->setInverseKinematics( Robotis->cnt , Robotis->ik_start_rotation );

            int max_iter = 30;
            double ik_tol = 1e-3;
            bool ik_success = Humanoid->InverseKinematics( Robotis->ik_id_start , Robotis->ik_id_end ,
                                                           Robotis->ik_target_position , Robotis->ik_target_rotation ,
                                                           max_iter , ik_tol , Robotis->ik_weight );

            if ( ik_success == true )
            {
                for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                    JointState->goal_joint_state[ id ].position = Humanoid->thormang3_link_data[ id ]->joint_angle;
            }
            else
            {
            	ROS_INFO("----- ik failed -----");
            	ROS_INFO("[end] send trajectory");

                PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

                Robotis->is_moving = false;
                Robotis->ik_solve = false;
                Robotis->cnt = 0;
            }
        }
        else
        {
            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                JointState->goal_joint_state[ id ].position = Robotis->calc_joint_tra( Robotis->cnt , id );
        }

        Robotis->cnt++;
    }

    /*----- set joint data -----*/
    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;
        result[_joint_name]->goal_position = JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position;
    }

    /*---------- initialize count number ----------*/
    if ( Robotis->is_moving == true )
    {
        if ( Robotis->cnt >= Robotis->all_time_steps )
        {
            ROS_INFO("[end] send trajectory");

            PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

            Robotis->is_moving = false;
            Robotis->ik_solve = false;
            Robotis->cnt = 0;
        }
    }

}

void ManipulationModule::Stop()
{
    Robotis->is_moving = false;
    Robotis->ik_solve = false;
    Robotis->cnt = 0;

    return;
}

bool ManipulationModule::IsRunning()
{
    return Robotis->is_moving;
}

void ManipulationModule::PublishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "Manipulation";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}
