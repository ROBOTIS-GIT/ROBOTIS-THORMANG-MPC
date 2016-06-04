/*
 * TestMotionModule.cpp
 *
 *  Created on: 2016. 1. 25.
 *      Author: zerom
 */

#include <stdio.h>
#include <eigen_conversions/eigen_msg.h>

#include "thormang3_walking_module/WalkingModule.h"

using namespace ROBOTIS;

WalkingMotionModule *WalkingMotionModule::unique_instance_ = new WalkingMotionModule();

WalkingMotionModule::WalkingMotionModule()
    : control_cycle_msec_(8)
{
    enable          = false;
    module_name     = "walking_module";
    control_mode    = POSITION_CONTROL;
    result["r_leg_hip_y"] = new DynamixelState();
    result["r_leg_hip_r"] = new DynamixelState();
    result["r_leg_hip_p"] = new DynamixelState();
    result["r_leg_kn_p"] = new DynamixelState();
    result["r_leg_an_p"] = new DynamixelState();
    result["r_leg_an_r"] = new DynamixelState();

    result["l_leg_hip_y"] = new DynamixelState();
    result["l_leg_hip_r"] = new DynamixelState();
    result["l_leg_hip_p"] = new DynamixelState();
    result["l_leg_kn_p" ] = new DynamixelState();
    result["l_leg_an_p" ] = new DynamixelState();
    result["l_leg_an_r" ] = new DynamixelState();

    previous_enable		= present_enable	= false;
    previous_running	= present_running	= false;

    gyro_x =  gyro_y = 0;
    orientation_roll = orientation_pitch = 0;
    r_foot_fx_N  = r_foot_fy_N  = r_foot_fz_N  = 0;
    r_foot_Tx_Nm = r_foot_Ty_Nm = r_foot_Tz_Nm = 0;
    l_foot_fx_N  = l_foot_fy_N  = l_foot_fz_N  = 0;
    l_foot_Tx_Nm = l_foot_Ty_Nm = l_foot_Tz_Nm = 0;


    r_foot_ft_publish_checker_ = -1;
    l_foot_ft_publish_checker_ = -1;

    desired_matrix_g_to_cob_   = Eigen::MatrixXd::Identity(4,4);
    desired_matrix_g_to_rfoot_ = Eigen::MatrixXd::Identity(4,4);
    desired_matrix_g_to_lfoot_ = Eigen::MatrixXd::Identity(4,4);


    balance_update_with_loop_ = false;
    balance_update_duration_ = 1.0;
    balance_update_sys_time_ = 1.0;
    balance_update_polynomial_coeff_.resize(6, 1);

    double tf = balance_update_duration_;
	Eigen::MatrixXd A(6,6), B(6, 1);
	A <<  0.0,	 0.0,	 0.0,	0.0,	0.0, 1.0,
		  0.0,   0.0,    0.0,	0.0,   	1.0, 0.0,
		  0.0,   0.0,    0.0,	2.0,   	0.0, 0.0,
	         tf*tf*tf*tf*tf,	 tf*tf*tf*tf,	  tf*tf*tf,	    tf*tf,	 tf, 1.0,
		 5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,	2.0*tf,    	1.0, 0.0,
		20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,		2.0,    	0.0, 0.0;

	B << 0, 0, 0, 1.0, 0, 0;

	balance_update_polynomial_coeff_ = A.inverse() * B;



	RX_PI_3x3.resize(3,3);
	RX_PI_3x3 << 1,  0,  0,
			 0, -1,  0,
			 0,  0, -1;

	RZ_PI_3x3.resize(3,3);
	RZ_PI_3x3 << -1, 0, 0,
			0, -1, 0,
			0,  0, 1;
}

WalkingMotionModule::~WalkingMotionModule()
{
    queue_thread_.join();
}

void WalkingMotionModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    queue_thread_ = boost::thread(boost::bind(&WalkingMotionModule::QueueThread, this));
    control_cycle_msec_ = control_cycle_msec;

    PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
    prev_walking->SetInitialPose(0, -93.0*0.001, -630*0.001, 0, 0, 0,
    		0, 93.0*0.001, -630*0.001,   0, 0, 0,
			0,  0,   0, 0, 0, 0);

    prev_walking->SetFTScaleFactor(1.0, 1.0);
    prev_walking->SetInitForceTorque(0, 0, 0, 0, 0, 0,
    									  0, 0, 0, 0, 0, 0);

    prev_walking->WALK_STABILIZER_GAIN_RATIO = 0;
	prev_walking->IMU_GYRO_GAIN_RATIO = 0.0;//7.31*0.01;
	prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO = 0.0;

	prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = 0.0*M_PI/180.0;
	////////// Damping Controller
	prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU = 0;
	prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = 0;

	prev_walking->BALANCE_X_GAIN_BY_FT = 0;
	prev_walking->BALANCE_Y_GAIN_BY_FT = 0;
	prev_walking->BALANCE_Z_GAIN_BY_FT = 0;


	prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT = 0;
	prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT = 0;

	prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT = 0;
	prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT = 0;

	//the below gain must be zero.
	prev_walking->BALANCE_HIP_ROLL_GAIN = 0;
	prev_walking->BALANCE_HIP_PITCH_GAIN = 0;
	prev_walking->AXIS_CONTROLLER_GAIN = 0;
	prev_walking->LANDING_CONTROLLER_GAIN = 0;

	//time constant
	prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU	= 0.2;
	prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU	= 0.2;

	prev_walking->BALANCE_X_TIME_CONSTANT					= 0.1;
	prev_walking->BALANCE_Y_TIME_CONSTANT					= 0.1;
	prev_walking->BALANCE_Z_TIME_CONSTANT					= 0.1;
	prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT	= 0.1;
	prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT	= 0.1;
	prev_walking->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT	 	= 0.1;
	prev_walking->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT	= 0.1;


	prev_walking->COB_X_MANUAL_ADJUSTMENT_M	= -10.0*0.001;

	prev_walking->Initialize();

	prev_walking->BALANCE_ENABLE = true;

	publish_mutex_.lock();
	desired_matrix_g_to_cob_   = prev_walking->matGtoCOB;
	desired_matrix_g_to_rfoot_ = prev_walking->matGtoRF;
	desired_matrix_g_to_lfoot_ = prev_walking->matGtoLF;
	publish_mutex_.unlock();

    result["r_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[0];
    result["r_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[1];
    result["r_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[2];
    result["r_leg_kn_p"]->goal_position  = prev_walking->m_OutAngleRad[3];
    result["r_leg_an_p"]->goal_position  = prev_walking->m_OutAngleRad[4];
    result["r_leg_an_r"]->goal_position  = prev_walking->m_OutAngleRad[5];

    result["l_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[6];
    result["l_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[7];
    result["l_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[8];
    result["l_leg_kn_p" ]->goal_position = prev_walking->m_OutAngleRad[9];
    result["l_leg_an_p" ]->goal_position = prev_walking->m_OutAngleRad[10];
    result["l_leg_an_r" ]->goal_position = prev_walking->m_OutAngleRad[11];

    prev_walking->Start();
    prev_walking->Process();

    previous_enable = enable;
    previous_running = IsRunning();
}

void	WalkingMotionModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* publish topics */
    robot_pose_pub_	= _ros_node.advertise<thormang3_walking_module_msgs::RobotPose>("/robotis/walking/robot_pose", 1);
    //status_msg_pub_	= _ros_node.advertise<std_msgs::String>("/robotis/walking/status_message", 1);
    status_msg_pub_ = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);


    /* ROS Service Callback Functions */
    ros::ServiceServer get_ref_step_data_server	 = _ros_node.advertiseService("/robotis/walking/get_reference_step_data",
    																			&WalkingMotionModule::GetReferenceStepDataServiceCallback, this);
    ros::ServiceServer add_step_data_array_sever = _ros_node.advertiseService("/robotis/walking/add_step_data",		&WalkingMotionModule::AddStepDataServiceCallback,  		this);
    ros::ServiceServer walking_start_server      = _ros_node.advertiseService("/robotis/walking/walking_start",		&WalkingMotionModule::WalkingStartServiceCallback,		this);
    ros::ServiceServer is_running_server   		 = _ros_node.advertiseService("/robotis/walking/is_running",   		&WalkingMotionModule::IsRunningServiceCallback, 		this);
    ros::ServiceServer set_balance_param_server	 = _ros_node.advertiseService("/robotis/walking/set_balance_param",	&WalkingMotionModule::SetBalanceParamServiceCallback,	this);
    ros::ServiceServer remove_existing_step_data = _ros_node.advertiseService("/robotis/walking/remove_existing_step_data",
    																			&WalkingMotionModule::RemoveExistingStepDataServiceCallback, this);

    /* sensor topic subscribe */
    ros::Subscriber imu_data_sub	= _ros_node.subscribe("/robotis/sensor/imu/imu",	3, &WalkingMotionModule::IMUDataOutputCallback,		this);

    ros::WallDuration duration(control_cycle_msec_/1000.0);
    while(_ros_node.ok())
      _callback_queue.callAvailable(duration);
}

void	WalkingMotionModule::PublishRobotPose(void)
{
	publish_mutex_.lock();
	robot_pose_msg_.global_to_center_of_body.position.x = desired_matrix_g_to_cob_.coeff(0, 3);
	robot_pose_msg_.global_to_center_of_body.position.y = desired_matrix_g_to_cob_.coeff(1, 3);
	robot_pose_msg_.global_to_center_of_body.position.z = desired_matrix_g_to_cob_.coeff(2, 3);
	Eigen::Quaterniond quaterniond_g_to_cob(desired_matrix_g_to_cob_.block<3, 3>(0, 0));


	robot_pose_msg_.global_to_right_foot.position.x = desired_matrix_g_to_rfoot_.coeff(0, 3);
	robot_pose_msg_.global_to_right_foot.position.y = desired_matrix_g_to_rfoot_.coeff(1, 3);
	robot_pose_msg_.global_to_right_foot.position.z = desired_matrix_g_to_rfoot_.coeff(2, 3);
	Eigen::Quaterniond quaterniond_g_to_rf(desired_matrix_g_to_rfoot_.block<3, 3>(0, 0));

	robot_pose_msg_.global_to_left_foot.position.x = desired_matrix_g_to_lfoot_.coeff(0, 3);
	robot_pose_msg_.global_to_left_foot.position.y = desired_matrix_g_to_lfoot_.coeff(1, 3);
	robot_pose_msg_.global_to_left_foot.position.z = desired_matrix_g_to_lfoot_.coeff(2, 3);
	Eigen::Quaterniond quaterniond_g_to_lf(desired_matrix_g_to_lfoot_.block<3, 3>(0, 0));
	publish_mutex_.unlock();

	tf::quaternionEigenToMsg(quaterniond_g_to_cob, robot_pose_msg_.global_to_center_of_body.orientation);
	tf::quaternionEigenToMsg(quaterniond_g_to_rf,  robot_pose_msg_.global_to_right_foot.orientation);
	tf::quaternionEigenToMsg(quaterniond_g_to_lf,  robot_pose_msg_.global_to_left_foot.orientation);

	robot_pose_pub_.publish(robot_pose_msg_);

}


void	WalkingMotionModule::PublishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "Walking";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}


int		WalkingMotionModule::StepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, StepData& des)
{
	int copy_result = STEP_DATA_ERR::NO_ERROR;
	des.TimeData.bWalkingState			= src.time_data.walking_state;
	des.TimeData.dAbsStepTime			= src.time_data.abs_step_time;
	des.TimeData.dDSPratio				= src.time_data.dsp_ratio;
	des.TimeData.sigmoid_ratio_x		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_x;
	des.TimeData.sigmoid_ratio_y		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_y;
	des.TimeData.sigmoid_ratio_z		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_z;
	des.TimeData.sigmoid_ratio_roll		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_roll;
	des.TimeData.sigmoid_ratio_pitch	= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_pitch;
	des.TimeData.sigmoid_ratio_yaw		= 1.0;//msg->step_data[i].TimeData.front_pause_ratio_yaw;

	des.TimeData.sigmoid_distortion_x		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_x;
	des.TimeData.sigmoid_distortion_y		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_y;
	des.TimeData.sigmoid_distortion_z		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_z;
	des.TimeData.sigmoid_distortion_roll	= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_roll;
	des.TimeData.sigmoid_distortion_pitch	= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_pitch;
	des.TimeData.sigmoid_distortion_yaw		= 1.0;//msg->step_data[i].TimeData.back_pause_ratio_yaw;


	des.PositionData.bMovingFoot		= src.position_data.moving_foot;
	des.PositionData.dShoulderSwingGain	= 0;
	des.PositionData.dElbowSwingGain	= 0;
	des.PositionData.dFootHeight		= src.position_data.foot_z_swap;
	des.PositionData.dWaistPitchAngle	= 0;
	des.PositionData.dWaistYawAngle		= src.position_data.torso_yaw_angle_rad;
	des.PositionData.dZ_Swap_Amplitude	= src.position_data.body_z_swap;

	des.PositionData.stBodyPosition.z			= src.position_data.body_pose.z;
	des.PositionData.stBodyPosition.roll		= src.position_data.body_pose.roll;
	des.PositionData.stBodyPosition.pitch		= src.position_data.body_pose.pitch;
	des.PositionData.stBodyPosition.yaw			= src.position_data.body_pose.yaw;
	des.PositionData.stRightFootPosition.x		= src.position_data.right_foot_pose.x;
	des.PositionData.stRightFootPosition.y		= src.position_data.right_foot_pose.y;
	des.PositionData.stRightFootPosition.z		= src.position_data.right_foot_pose.z;
	des.PositionData.stRightFootPosition.roll	= src.position_data.right_foot_pose.roll;
	des.PositionData.stRightFootPosition.pitch	= src.position_data.right_foot_pose.pitch;
	des.PositionData.stRightFootPosition.yaw	= src.position_data.right_foot_pose.yaw;
	des.PositionData.stLeftFootPosition.x		= src.position_data.left_foot_pose.x;
	des.PositionData.stLeftFootPosition.y		= src.position_data.left_foot_pose.y;
	des.PositionData.stLeftFootPosition.z		= src.position_data.left_foot_pose.z;
	des.PositionData.stLeftFootPosition.roll	= src.position_data.left_foot_pose.roll;
	des.PositionData.stLeftFootPosition.pitch	= src.position_data.left_foot_pose.pitch;
	des.PositionData.stLeftFootPosition.yaw		= src.position_data.left_foot_pose.yaw;

	if((src.time_data.walking_state != WalkingStateFlag::InWalkingStarting)
		&& (src.time_data.walking_state != WalkingStateFlag::InWalking)
		&& (src.time_data.walking_state != WalkingStateFlag::InWalkingEnding) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if((src.time_data.start_time_delay_ratio_x < 0)
		|| (src.time_data.start_time_delay_ratio_y < 0)
		|| (src.time_data.start_time_delay_ratio_z < 0)
		|| (src.time_data.start_time_delay_ratio_roll < 0)
		|| (src.time_data.start_time_delay_ratio_pitch < 0)
		|| (src.time_data.start_time_delay_ratio_yaw < 0) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if((src.time_data.finish_time_advance_ratio_x < 0)
		|| (src.time_data.finish_time_advance_ratio_y < 0)
		|| (src.time_data.finish_time_advance_ratio_z < 0)
		|| (src.time_data.finish_time_advance_ratio_roll < 0)
		|| (src.time_data.finish_time_advance_ratio_pitch < 0)
		|| (src.time_data.finish_time_advance_ratio_yaw < 0) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_y		+ src.time_data.finish_time_advance_ratio_y		) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_z		+ src.time_data.finish_time_advance_ratio_z		) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_roll	+ src.time_data.finish_time_advance_ratio_roll	) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_pitch	+ src.time_data.finish_time_advance_ratio_pitch	) > 1.0)
		|| ((src.time_data.start_time_delay_ratio_yaw	+ src.time_data.finish_time_advance_ratio_yaw	) > 1.0) )
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

	if((src.position_data.moving_foot != MovingFootFlag::NFootMove)
		&& (src.position_data.moving_foot != MovingFootFlag::RFootMove)
		&& (src.position_data.moving_foot != MovingFootFlag::LFootMove))
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA;

	if(src.position_data.foot_z_swap < 0)
		copy_result |= STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA;

	return copy_result;
}

int		WalkingMotionModule::StepDataToStepDataMsg(StepData& src, thormang3_walking_module_msgs::StepData& des)
{
	des.time_data.walking_state   = src.TimeData.bWalkingState;
	des.time_data.abs_step_time   = src.TimeData.dAbsStepTime;
	des.time_data.dsp_ratio       = src.TimeData.dDSPratio;

	des.time_data.start_time_delay_ratio_x		= des.time_data.finish_time_advance_ratio_x		= 0;
	des.time_data.start_time_delay_ratio_y		= des.time_data.finish_time_advance_ratio_y		= 0;
	des.time_data.start_time_delay_ratio_z		= des.time_data.finish_time_advance_ratio_z		= 0;
	des.time_data.start_time_delay_ratio_roll	= des.time_data.finish_time_advance_ratio_roll	= 0;
	des.time_data.start_time_delay_ratio_pitch	= des.time_data.finish_time_advance_ratio_pitch	= 0;
	des.time_data.start_time_delay_ratio_yaw	= des.time_data.finish_time_advance_ratio_yaw	= 0;

	des.position_data.moving_foot          		= src.PositionData.bMovingFoot;
	des.position_data.foot_z_swap          		= src.PositionData.dFootHeight;
	des.position_data.torso_yaw_angle_rad  		= src.PositionData.dWaistYawAngle;
	des.position_data.body_z_swap          		= src.PositionData.dZ_Swap_Amplitude;

	des.position_data.body_pose.z             = src.PositionData.stBodyPosition.z;
	des.position_data.body_pose.roll          = src.PositionData.stBodyPosition.roll;
	des.position_data.body_pose.pitch         = src.PositionData.stBodyPosition.pitch;
	des.position_data.body_pose.yaw           = src.PositionData.stBodyPosition.yaw;
	des.position_data.right_foot_pose.x       = src.PositionData.stRightFootPosition.x;
	des.position_data.right_foot_pose.y       = src.PositionData.stRightFootPosition.y;
	des.position_data.right_foot_pose.z       = src.PositionData.stRightFootPosition.z;
	des.position_data.right_foot_pose.roll    = src.PositionData.stRightFootPosition.roll;
	des.position_data.right_foot_pose.pitch   = src.PositionData.stRightFootPosition.pitch;
	des.position_data.right_foot_pose.yaw     = src.PositionData.stRightFootPosition.yaw;
	des.position_data.left_foot_pose.x        = src.PositionData.stLeftFootPosition.x;
	des.position_data.left_foot_pose.y        = src.PositionData.stLeftFootPosition.y;
	des.position_data.left_foot_pose.z        = src.PositionData.stLeftFootPosition.z;
	des.position_data.left_foot_pose.roll     = src.PositionData.stLeftFootPosition.roll;
	des.position_data.left_foot_pose.pitch    = src.PositionData.stLeftFootPosition.pitch;
	des.position_data.left_foot_pose.yaw      = src.PositionData.stLeftFootPosition.yaw;

	return 0;
}


bool 	WalkingMotionModule::GetReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request &req,
																thormang3_walking_module_msgs::GetReferenceStepData::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	StepData _refStepData;

	prev_walking->GetReferenceStepDatafotAddition(&_refStepData);

	StepDataToStepDataMsg(_refStepData, res.reference_step_data);

	return true;
}




bool 	WalkingMotionModule::AddStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request &req,
														thormang3_walking_module_msgs::AddStepDataArray::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
	res.result = STEP_DATA_ERR::NO_ERROR;

	if(enable == false) {
		res.result |= STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE;
		std::string _status_msg = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
		//status_msg_pub_.publish(status_msg);
		return true;
	}

	if(prev_walking->IsRunning() == true) {
		res.result |= STEP_DATA_ERR::ROBOT_IS_WALKING_NOW;
		std::string _status_msg  = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
		return true;
	}

	StepData _stepData, _refStepData;
	std::vector<StepData> _req_stp_data_array;

	prev_walking->GetReferenceStepDatafotAddition(&_refStepData);

	for(int i = 0; i < req.step_data_array.size(); i++)
	{
		res.result |= StepDataMsgToStepData(req.step_data_array[i], _stepData);

		if(_stepData.TimeData.dAbsStepTime <= 0) {
			res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
		}

		if(i != 0) {
			if(_stepData.TimeData.dAbsStepTime <= _req_stp_data_array[_req_stp_data_array.size() - 1].TimeData.dAbsStepTime) {
				res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
			}
		}
		else {
			if(_stepData.TimeData.dAbsStepTime <= _refStepData.TimeData.dAbsStepTime) {
				res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
			}
		}

		if(res.result != STEP_DATA_ERR::NO_ERROR) {
			std::string _status_msg = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
			return true;
		}

		_req_stp_data_array.push_back(_stepData);
	}


	if(req.remove_existing_step_data == true) {
		int _exist_num_of_step_data = prev_walking->GetNumofRemainingUnreservedStepData();
		if(_exist_num_of_step_data != 0)
			for(int _remove_count  = 0; _remove_count < _exist_num_of_step_data; _remove_count++)
				prev_walking->EraseLastStepData();
	}

	for(unsigned int _i = 0; _i <_req_stp_data_array.size() ; _i++)
		prev_walking->AddStepData(_req_stp_data_array[_i]);

	if( req.auto_start == true)	{
		prev_walking->Start();
	}

	return true;
}

bool	WalkingMotionModule::WalkingStartServiceCallback(thormang3_walking_module_msgs::WalkingStart::Request &req,
														thormang3_walking_module_msgs::WalkingStart::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
	res.result = WALKING_START_ERR::NO_ERROR;

	if(enable == false) {
		res.result |= WALKING_START_ERR::NOT_ENABLED_WALKING_MODULE;
	}

	if(prev_walking->IsRunning() == true){
		res.result |= WALKING_START_ERR::ROBOT_IS_WALKING_NOW;
	}

	if(prev_walking->GetNumofRemainingUnreservedStepData() == 0){
		res.result |= WALKING_START_ERR::NO_STEP_DATA;
	}

	if(res.result == WALKING_START_ERR::NO_ERROR) {
		prev_walking->Start();
	}

	return true;
}

bool	WalkingMotionModule::IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request &req,
														thormang3_walking_module_msgs::IsRunning::Response &res)
{
	bool is_running = IsRunning();
	res.is_running = is_running;

	return true;
}

bool	WalkingMotionModule::IsRunning()
{
	return PreviewControlWalking::GetInstance()->IsRunning();
}

bool	WalkingMotionModule::SetBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
															thormang3_walking_module_msgs::SetBalanceParam::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();
	res.result = BALANCE_PARAM_ERR::NO_ERROR;

	if( enable == false)
		res.result |= BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE;

	if( balance_update_with_loop_ == true)	{
		res.result |= BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED;
	}

	if( ( req.balance_param.foot_roll_angle_time_constant  <= 0.0 )
			|| ( req.balance_param.foot_pitch_angle_time_constant  <= 0.0 )
			|| ( req.balance_param.foot_x_force_time_constant      <= 0.0 )
			|| ( req.balance_param.foot_y_force_time_constant      <= 0.0 )
			|| ( req.balance_param.foot_z_force_time_constant      <= 0.0 )
			|| ( req.balance_param.foot_roll_torque_time_constant  <= 0.0 )
			|| ( req.balance_param.foot_pitch_torque_time_constant <= 0.0 ) )
	{
		res.result |= BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE;
	}

	if(res.result == BALANCE_PARAM_ERR::NO_ERROR) {
		if( req.updating_duration < 0.0 )
		{
			// under 8ms apply immediately
			SetBalanceParam(req.balance_param);
			std::string _status_msg = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_FINISH_MSG;
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
			return true;
		}
		else if( req.updating_duration < (control_cycle_msec_ * 0.001) ) {
			balance_update_duration_ = 2.0;
		}
		else {
			balance_update_duration_ = req.updating_duration;
		}

		balance_update_sys_time_ = 0.0;
		balance_update_polynomial_coeff_.resize(6, 1);

		double tf = balance_update_duration_;
		Eigen::MatrixXd A(6,6), B(6, 1);
		A <<  0.0,	 0.0,	 0.0,	0.0,	0.0, 1.0,
				0.0,   0.0,    0.0,	0.0,   	1.0, 0.0,
				0.0,   0.0,    0.0,	2.0,   	0.0, 0.0,
				tf*tf*tf*tf*tf,	 tf*tf*tf*tf,	  tf*tf*tf,	    tf*tf,	 tf, 1.0,
				5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,	2.0*tf,    	1.0, 0.0,
				20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,		2.0,    	0.0, 0.0;

		B << 0, 0, 0, 1.0, 0, 0;
		balance_update_polynomial_coeff_ = A.inverse() * B;

		desired_balance_param_ = req.balance_param;

		previous_balance_param_.cob_x_offset_m					= prev_walking->COB_X_MANUAL_ADJUSTMENT_M;
		previous_balance_param_.cob_y_offset_m					= prev_walking->COB_Y_MANUAL_ADJUSTMENT_M;

		previous_balance_param_.hip_roll_swap_angle_rad 		= prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD;

		previous_balance_param_.gyro_gain 						= prev_walking->WALK_STABILIZER_GAIN_RATIO;
		previous_balance_param_.foot_roll_angle_gain            = prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU;
		previous_balance_param_.foot_pitch_angle_gain           = prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU;

		previous_balance_param_.foot_x_force_gain               = prev_walking->BALANCE_X_GAIN_BY_FT;
		previous_balance_param_.foot_y_force_gain               = prev_walking->BALANCE_Y_GAIN_BY_FT;
		previous_balance_param_.foot_z_force_gain               = prev_walking->BALANCE_Z_GAIN_BY_FT;
		previous_balance_param_.foot_roll_torque_gain           = prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT;
		previous_balance_param_.foot_pitch_torque_gain          = prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT;


		previous_balance_param_.foot_roll_angle_time_constant   = prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU;
		previous_balance_param_.foot_pitch_angle_time_constant  = prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU;

		previous_balance_param_.foot_x_force_time_constant      = prev_walking->BALANCE_X_TIME_CONSTANT;
		previous_balance_param_.foot_y_force_time_constant      = prev_walking->BALANCE_Y_TIME_CONSTANT;
		previous_balance_param_.foot_z_force_time_constant      = prev_walking->BALANCE_Z_TIME_CONSTANT;
		previous_balance_param_.foot_roll_torque_time_constant  = prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT;
		previous_balance_param_.foot_pitch_torque_time_constant = prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT;

		balance_update_with_loop_ = true;

		std::string _status_msg = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_START_MSG;
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
	}

	return true;
}
//
void	WalkingMotionModule::SetBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	prev_walking->COB_X_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_x_offset_m;
	prev_walking->COB_Y_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_y_offset_m;
	prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = balance_param_msg.hip_roll_swap_angle_rad;

	prev_walking->WALK_STABILIZER_GAIN_RATIO = balance_param_msg.gyro_gain;
	prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
	prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

	prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = balance_param_msg.foot_roll_angle_gain;
	prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = balance_param_msg.foot_pitch_angle_gain;

	prev_walking->BALANCE_X_GAIN_BY_FT				= balance_param_msg.foot_x_force_gain;
	prev_walking->BALANCE_Y_GAIN_BY_FT				= balance_param_msg.foot_y_force_gain;
	prev_walking->BALANCE_Z_GAIN_BY_FT				= balance_param_msg.foot_z_force_gain;
	prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT		= balance_param_msg.foot_roll_torque_gain;
	prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT		= balance_param_msg.foot_roll_torque_gain;
	prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT	= balance_param_msg.foot_pitch_torque_gain;
	prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT		= balance_param_msg.foot_pitch_torque_gain;

	prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU  = balance_param_msg.foot_roll_angle_time_constant;
	prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU = balance_param_msg.foot_pitch_angle_time_constant;

	prev_walking->BALANCE_X_TIME_CONSTANT					= balance_param_msg.foot_x_force_time_constant;
	prev_walking->BALANCE_Y_TIME_CONSTANT					= balance_param_msg.foot_y_force_time_constant;
	prev_walking->BALANCE_Z_TIME_CONSTANT					= balance_param_msg.foot_z_force_time_constant;
	prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT	= balance_param_msg.foot_roll_torque_time_constant;
	prev_walking->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT		= balance_param_msg.foot_roll_torque_time_constant;
	prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT	= balance_param_msg.foot_pitch_torque_time_constant;
	prev_walking->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT	= balance_param_msg.foot_pitch_torque_time_constant;
}

bool	WalkingMotionModule::RemoveExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
																	thormang3_walking_module_msgs::RemoveExistingStepData::Response &res)
{
	PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	res.result = REMOVE_STEP_DATA_ERR::NO_ERROR;

	if(IsRunning())
		res.result |= REMOVE_STEP_DATA_ERR::ROBOT_IS_WALKING_NOW;
	else {
		int _exist_num_of_step_data = prev_walking->GetNumofRemainingUnreservedStepData();
		if(_exist_num_of_step_data != 0)
			for(int _remove_count  = 0; _remove_count < _exist_num_of_step_data; _remove_count++)
				prev_walking->EraseLastStepData();
	}
	return true;
}


void	WalkingMotionModule::IMUDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	PreviewControlWalking::GetInstance()->current_gyro_roll_rad_per_sec  = -1.0*(msg->angular_velocity.x);
	PreviewControlWalking::GetInstance()->current_gyro_pitch_rad_per_sec = -1.0*(msg->angular_velocity.y);


	Eigen::Quaterniond imu_quat;
	tf::quaternionMsgToEigen(msg->orientation, imu_quat);

	Eigen::MatrixXd imu_mat = (RX_PI_3x3*(imu_quat.toRotationMatrix()))*RZ_PI_3x3;

	double roll  = atan2( imu_mat.coeff(2,1), imu_mat.coeff(2,2));
	double pitch = atan2(-imu_mat.coeff(2,0), sqrt(powDI(imu_mat.coeff(2,1), 2) + powDI(imu_mat.coeff(2,2), 2)));
	double yaw   = atan2( imu_mat.coeff(1,0), imu_mat.coeff(0,0));

	PreviewControlWalking::GetInstance()->current_imu_roll_rad = roll;
	PreviewControlWalking::GetInstance()->current_imu_pitch_rad = pitch;
}

void	WalkingMotionModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
	present_enable = enable;
    PreviewControlWalking *prev_walking = PreviewControlWalking::GetInstance();

	if(previous_enable != present_enable)
	{
		std::string _status_msg;
		if(present_enable == true)
		{
			_status_msg = WALKING_STATUS_MSG::WALKING_MODULE_IS_ENABLED_MSG;
		}
		else {
			_status_msg = WALKING_STATUS_MSG::WALKING_MODULE_IS_DISABLED_MSG;
			balance_update_with_loop_ = false;

			prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = 0.0;

			prev_walking->WALK_STABILIZER_GAIN_RATIO = 0.0;
			prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
			prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
			prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

			prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
			prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

			prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = 0.0;
			prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = 0.0;

			prev_walking->BALANCE_X_GAIN_BY_FT				= 0.0;
			prev_walking->BALANCE_Y_GAIN_BY_FT				= 0.0;
			prev_walking->BALANCE_Z_GAIN_BY_FT				= 0.0;
			prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT		= 0.0;
			prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT		= 0.0;
			prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT	= 0.0;
			prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT		= 0.0;

		}

		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
	}
	previous_enable = present_enable;

	r_foot_fx_N  = sensors["r_foot_fx_scaled_N"];
	r_foot_fy_N  = sensors["r_foot_fy_scaled_N"];
	r_foot_fz_N	 = sensors["r_foot_fz_scaled_N"];
	r_foot_Tx_Nm = sensors["r_foot_tx_scaled_Nm"];
	r_foot_Ty_Nm = sensors["r_foot_ty_scaled_Nm"];
	r_foot_Tz_Nm = sensors["r_foot_tz_scaled_Nm"];

	l_foot_fx_N  = sensors["l_foot_fx_scaled_N"];
	l_foot_fy_N  = sensors["l_foot_fy_scaled_N"];
	l_foot_fz_N	 = sensors["l_foot_fz_scaled_N"];
	l_foot_Tx_Nm = sensors["l_foot_tx_scaled_Nm"];
	l_foot_Ty_Nm = sensors["l_foot_ty_scaled_Nm"];
	l_foot_Tz_Nm = sensors["l_foot_tz_scaled_Nm"];


	r_foot_fx_N = sign(r_foot_fx_N) * fmin( fabs(r_foot_fx_N), 2000.0);
	r_foot_fy_N = sign(r_foot_fy_N) * fmin( fabs(r_foot_fy_N), 2000.0);
	r_foot_fz_N = sign(r_foot_fz_N) * fmin( fabs(r_foot_fz_N), 2000.0);
	r_foot_Tx_Nm = sign(r_foot_Tx_Nm) *fmin(fabs(r_foot_Tx_Nm), 300.0);
	r_foot_Ty_Nm = sign(r_foot_Ty_Nm) *fmin(fabs(r_foot_Ty_Nm), 300.0);
	r_foot_Tz_Nm = sign(r_foot_Tz_Nm) *fmin(fabs(r_foot_Tz_Nm), 300.0);

	l_foot_fx_N = sign(l_foot_fx_N) * fmin( fabs(l_foot_fx_N), 2000.0);
	l_foot_fy_N = sign(l_foot_fy_N) * fmin( fabs(l_foot_fy_N), 2000.0);
	l_foot_fz_N = sign(l_foot_fz_N) * fmin( fabs(l_foot_fz_N), 2000.0);
	l_foot_Tx_Nm = sign(l_foot_Tx_Nm) *fmin(fabs(l_foot_Tx_Nm), 300.0);
	l_foot_Ty_Nm = sign(l_foot_Ty_Nm) *fmin(fabs(l_foot_Ty_Nm), 300.0);
	l_foot_Tz_Nm = sign(l_foot_Tz_Nm) *fmin(fabs(l_foot_Tz_Nm), 300.0);


    if(enable == false)
        return;


    if(balance_update_with_loop_ == true)
    {
    	balance_update_sys_time_ += control_cycle_msec_ * 0.001;
    	if(balance_update_sys_time_ >= balance_update_duration_ ) {
    		balance_update_sys_time_ = balance_update_duration_;
    		balance_update_with_loop_ = false;
    		SetBalanceParam(desired_balance_param_);
    		std::string _status_msg = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_FINISH_MSG;
    		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
    	}
    	else {
    		double current_update_gain =  balance_update_polynomial_coeff_.coeff(0,0) * powDI(balance_update_sys_time_ , 5)
    									+ balance_update_polynomial_coeff_.coeff(1,0) * powDI(balance_update_sys_time_ , 4)
										+ balance_update_polynomial_coeff_.coeff(2,0) * powDI(balance_update_sys_time_ , 3)
										+ balance_update_polynomial_coeff_.coeff(3,0) * powDI(balance_update_sys_time_ , 2)
										+ balance_update_polynomial_coeff_.coeff(4,0) * powDI(balance_update_sys_time_ , 1)
										+ balance_update_polynomial_coeff_.coeff(5,0) ;

    		current_balance_param_.cob_x_offset_m                  = current_update_gain*(desired_balance_param_.cob_x_offset_m      			 - previous_balance_param_.cob_x_offset_m			      ) + previous_balance_param_.cob_x_offset_m;
    		current_balance_param_.cob_y_offset_m                  = current_update_gain*(desired_balance_param_.cob_y_offset_m      			 - previous_balance_param_.cob_y_offset_m			      ) + previous_balance_param_.cob_y_offset_m;

    		current_balance_param_.hip_roll_swap_angle_rad		   = current_update_gain*(desired_balance_param_.hip_roll_swap_angle_rad         - previous_balance_param_.hip_roll_swap_angle_rad        ) + previous_balance_param_.hip_roll_swap_angle_rad;

    		current_balance_param_.gyro_gain					   = current_update_gain*(desired_balance_param_.gyro_gain						 - previous_balance_param_.gyro_gain					  ) + previous_balance_param_.gyro_gain;
    		current_balance_param_.foot_roll_angle_gain            = current_update_gain*(desired_balance_param_.foot_roll_angle_gain            - previous_balance_param_.foot_roll_angle_gain           ) + previous_balance_param_.foot_roll_angle_gain;
    		current_balance_param_.foot_pitch_angle_gain           = current_update_gain*(desired_balance_param_.foot_pitch_angle_gain           - previous_balance_param_.foot_pitch_angle_gain          ) + previous_balance_param_.foot_pitch_angle_gain;

    		current_balance_param_.foot_x_force_gain               = current_update_gain*(desired_balance_param_.foot_x_force_gain               - previous_balance_param_.foot_x_force_gain              ) + previous_balance_param_.foot_x_force_gain;
    		current_balance_param_.foot_y_force_gain               = current_update_gain*(desired_balance_param_.foot_y_force_gain               - previous_balance_param_.foot_y_force_gain              ) + previous_balance_param_.foot_y_force_gain;
    		current_balance_param_.foot_z_force_gain               = current_update_gain*(desired_balance_param_.foot_z_force_gain               - previous_balance_param_.foot_z_force_gain              ) + previous_balance_param_.foot_z_force_gain;
    		current_balance_param_.foot_roll_torque_gain           = current_update_gain*(desired_balance_param_.foot_roll_torque_gain           - previous_balance_param_.foot_roll_torque_gain          ) + previous_balance_param_.foot_roll_torque_gain;
    		current_balance_param_.foot_pitch_torque_gain          = current_update_gain*(desired_balance_param_.foot_pitch_torque_gain          - previous_balance_param_.foot_pitch_torque_gain         ) + previous_balance_param_.foot_pitch_torque_gain;


    		current_balance_param_.foot_roll_angle_time_constant   = current_update_gain*(desired_balance_param_.foot_roll_angle_time_constant   - previous_balance_param_.foot_roll_angle_time_constant  ) + previous_balance_param_.foot_roll_angle_time_constant;
    		current_balance_param_.foot_pitch_angle_time_constant  = current_update_gain*(desired_balance_param_.foot_pitch_angle_time_constant  - previous_balance_param_.foot_pitch_angle_time_constant ) + previous_balance_param_.foot_pitch_angle_time_constant;

    		current_balance_param_.foot_x_force_time_constant      = current_update_gain*(desired_balance_param_.foot_x_force_time_constant      - previous_balance_param_.foot_x_force_time_constant     ) + previous_balance_param_.foot_x_force_time_constant;
    		current_balance_param_.foot_y_force_time_constant      = current_update_gain*(desired_balance_param_.foot_y_force_time_constant      - previous_balance_param_.foot_y_force_time_constant     ) + previous_balance_param_.foot_y_force_time_constant;
    		current_balance_param_.foot_z_force_time_constant      = current_update_gain*(desired_balance_param_.foot_z_force_time_constant      - previous_balance_param_.foot_z_force_time_constant     ) + previous_balance_param_.foot_z_force_time_constant;
    		current_balance_param_.foot_roll_torque_time_constant  = current_update_gain*(desired_balance_param_.foot_roll_torque_time_constant  - previous_balance_param_.foot_roll_torque_time_constant ) + previous_balance_param_.foot_roll_torque_time_constant;
    		current_balance_param_.foot_pitch_torque_time_constant = current_update_gain*(desired_balance_param_.foot_pitch_torque_time_constant - previous_balance_param_.foot_pitch_torque_time_constant) + previous_balance_param_.foot_pitch_torque_time_constant;


    		SetBalanceParam(current_balance_param_);
    	}
    }

    prev_walking->current_right_fx_N  = r_foot_fx_N;
    prev_walking->current_right_fy_N  = r_foot_fy_N;
    prev_walking->current_right_fz_N  = r_foot_fz_N;
    prev_walking->current_right_Tx_Nm = r_foot_Tx_Nm;
    prev_walking->current_right_Ty_Nm = r_foot_Ty_Nm;
    prev_walking->current_right_Tz_Nm = r_foot_Tz_Nm;

    prev_walking->current_left_fx_N  = l_foot_fx_N;
    prev_walking->current_left_fy_N  = l_foot_fy_N;
    prev_walking->current_left_fz_N  = l_foot_fz_N;
    prev_walking->current_left_Tx_Nm = l_foot_Tx_Nm;
    prev_walking->current_left_Ty_Nm = l_foot_Ty_Nm;
    prev_walking->current_left_Tz_Nm = l_foot_Tz_Nm;


	prev_walking->Process();

	publish_mutex_.lock();
	desired_matrix_g_to_cob_   = prev_walking->matGtoCOB;
	desired_matrix_g_to_rfoot_ = prev_walking->matGtoRF;
	desired_matrix_g_to_lfoot_ = prev_walking->matGtoLF;
	publish_mutex_.unlock();

    result["r_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[0];
    result["r_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[1];
    result["r_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[2];
    result["r_leg_kn_p" ]->goal_position = prev_walking->m_OutAngleRad[3];
    result["r_leg_an_p" ]->goal_position = prev_walking->m_OutAngleRad[4];
    result["r_leg_an_r" ]->goal_position = prev_walking->m_OutAngleRad[5];

    result["l_leg_hip_y"]->goal_position = prev_walking->m_OutAngleRad[6];
    result["l_leg_hip_r"]->goal_position = prev_walking->m_OutAngleRad[7];
    result["l_leg_hip_p"]->goal_position = prev_walking->m_OutAngleRad[8];
    result["l_leg_kn_p" ]->goal_position = prev_walking->m_OutAngleRad[9];
    result["l_leg_an_p" ]->goal_position = prev_walking->m_OutAngleRad[10];
    result["l_leg_an_r" ]->goal_position = prev_walking->m_OutAngleRad[11];

    present_running = IsRunning();
    if(previous_running != present_running) {
    	if(present_running == true) {
    		std::string _status_msg = WALKING_STATUS_MSG::WALKING_START_MSG;
    		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
    	}
    	else {
    		std::string _status_msg = WALKING_STATUS_MSG::WALKING_FINISH_MSG;
    		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
    	}
    }
    previous_running = present_running;
}

void WalkingMotionModule::Stop()
{
	return;
}

