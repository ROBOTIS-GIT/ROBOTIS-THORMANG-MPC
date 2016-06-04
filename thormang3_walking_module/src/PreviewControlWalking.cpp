#include "thormang3_walking_module/PreviewControlWalking.h"
#include <cmath>
#include <iostream>


using namespace ROBOTIS;


static const double MMtoM = 0.001;
static const double MStoS = 0.001;

#define NO_STEP_IDX	(-1)
#define G  			(9810.0*MMtoM) // mm/s^2(= 9.81m/s^2 *1000mm/1m)
#define TIME_UNIT 	(8*MStoS)


FILE *fpPreviewControlWalking;
FILE *fpAxisController;
FILE *fpBalance;


static const int InWalkingStarting = 0;
static const int InWalking = 1;
static const int InWalkingEnding = 2;

static const int LFootMove = 1;
static const int RFootMove = 2;
static const int NFootMove = 3;


enum
{
	BALANCING_PHASE0 = 0,
	BALANCING_PHASE1 = 1,
	BALANCING_PHASE2 = 2,
	BALANCING_PHASE3 = 3,
	BALANCING_PHASE4 = 4,
	BALANCING_PHASE5 = 5,
	BALANCING_PHASE6 = 6,
	BALANCING_PHASE7 = 7,
	BALANCING_PHASE8 = 8,
	BALANCING_PHASE9 = 9
};

enum StepDataStatus
{
	StepDataStatus1 = 1, //
	StepDataStatus2 = 2, //
	StepDataStatus3 = 3, //
	StepDataStatus4 = 4, //
};

PreviewControlWalking::PreviewControlWalking()
{

	//uID = (char*)"PreviewControlWalking";

	thormang3_kd_ = new ROBOTIS::ThorMang3KinematicsDynamics(ROBOTIS::WHOLE_BODY);

	m_PresentRightFootPosition.x = 0.0;    m_PresentRightFootPosition.y = -0.5*186.0*MMtoM;  m_PresentRightFootPosition.z = -0.630*MMtoM;
	m_PresentRightFootPosition.roll = 0.0; m_PresentRightFootPosition.pitch = 0.0; m_PresentRightFootPosition.yaw = 0.0;

	m_PresentLeftFootPosition.x = 0.0;    m_PresentLeftFootPosition.y = 0.5*186.0*MMtoM;   m_PresentLeftFootPosition.z = -0.630*MMtoM;
	m_PresentLeftFootPosition.roll = 0.0; m_PresentLeftFootPosition.pitch = 0.0; m_PresentLeftFootPosition.yaw = 0.0;

	m_PresentBodyPosition.x = 0.0;    m_PresentBodyPosition.y = 0.0;     m_PresentBodyPosition.z = 0.0;
	m_PresentBodyPosition.roll = 0.0; m_PresentBodyPosition.pitch = 0.0; m_PresentBodyPosition.yaw = 0;

	m_PreviousStepRightFootPosition	= m_PresentRightFootPosition;
	m_PreviousStepLeftFootPosition	= m_PresentLeftFootPosition;
	m_PreviousStepBodyPosition		= m_PresentBodyPosition;

	m_InitialRightFootPosition = m_PreviousStepRightFootPosition;
	m_InitialLeftFootPosition  = m_PreviousStepLeftFootPosition;
	m_InitialBodyPosition      = m_PreviousStepBodyPosition;

	matCOBtoRH = GetTranslationMatrix(0.0, -0.5*(186.0)*MMtoM, 0.0);
	matRHtoCOB = GetTranslationMatrix(0.0,  0.5*(186.0)*MMtoM, 0.0);
	matCOBtoLH = GetTranslationMatrix(0.0,  0.5*(186.0)*MMtoM, 0.0);
	matLHtoCOB = GetTranslationMatrix(0.0, -0.5*(186.0)*MMtoM, 0.0);

//	matRFtoRFT = GetOrientationMatrix(PI,0,0)*GetOrientationMatrix(0,0,PI);
//	matLFtoLFT = GetOrientationMatrix(PI,0,0);
	matRFtoRFT = GetOrientationMatrix(M_PI,0,0);
	matLFtoLFT = GetOrientationMatrix(M_PI,0,0);

	matGtoCOB = GetTransformMatrix(m_PresentBodyPosition.x, m_PresentBodyPosition.y, m_PresentBodyPosition.z,
			m_PresentBodyPosition.roll, m_PresentBodyPosition.pitch, m_PresentBodyPosition.yaw);

	matCOBtoG = GetTransformMatrixInverse(matGtoCOB);

	matGtoRF = GetTransformMatrix(m_PresentRightFootPosition.x, m_PresentRightFootPosition.y, m_PresentRightFootPosition.z,
			m_PresentRightFootPosition.roll, m_PresentRightFootPosition.pitch, m_PresentRightFootPosition.yaw);
	matGtoLF = GetTransformMatrix(m_PresentLeftFootPosition.x, m_PresentLeftFootPosition.y, m_PresentLeftFootPosition.z,
			m_PresentLeftFootPosition.roll, m_PresentLeftFootPosition.pitch, m_PresentLeftFootPosition.yaw);


	matRHtoRF = matRHtoCOB*matCOBtoG*matGtoRF;
	matLHtoLF = matLHtoCOB*matCOBtoG*matGtoLF;


	m_GoalWaistYawAngleRad = 0.0*M_PI;
	m_ReferenceStepDataforAddition.PositionData.bMovingFoot = NFootMove;
	m_ReferenceStepDataforAddition.PositionData.dElbowSwingGain = 0.1;
	m_ReferenceStepDataforAddition.PositionData.dShoulderSwingGain = 0.05;
	m_ReferenceStepDataforAddition.PositionData.dFootHeight = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dWaistPitchAngle = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dWaistYawAngle = m_GoalWaistYawAngleRad;
	m_ReferenceStepDataforAddition.PositionData.dZ_Swap_Amplitude = 0.0;
	m_ReferenceStepDataforAddition.PositionData.stBodyPosition = m_PreviousStepBodyPosition;
	m_ReferenceStepDataforAddition.PositionData.stRightFootPosition = m_PreviousStepRightFootPosition;
	m_ReferenceStepDataforAddition.PositionData.stLeftFootPosition = m_PreviousStepLeftFootPosition;
	m_ReferenceStepDataforAddition.TimeData.bWalkingState = InWalkingEnding;
	m_ReferenceStepDataforAddition.TimeData.dAbsStepTime = 0.0;
	m_ReferenceStepDataforAddition.TimeData.dDSPratio = 0.2;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_yaw = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_yaw = 1.0;

	m_PresentWaistYawAngleRad = m_GoalWaistYawAngleRad;
	m_PreviousStepWaistYawAngleRad = m_GoalWaistYawAngleRad;

	m_CurrentStepDataStatus = StepDataStatus4;

	m_WalkingTime = 0; m_ReferenceTime = 0;
	m_Balancing_Index = 0;

	m_X_ZMP_CenterShift = 0.0;
	m_Y_ZMP_CenterShift = 0.0;
	m_Y_ZMP_Convergence = 0.0;

	m_PreviewTime = 1.6;
	m_PreviewSize = round(m_PreviewTime/TIME_UNIT);

	m_Real_Running = false; m_Ctrl_Running = false;

	m_ShoulerSwingGain	= 0.05;
	m_ElbowSwingGain 	= 0.1;

	dir[0] = -1;  dir[1] = -1; dir[2] =  1;   dir[3] = 1;  dir[4] = -1; dir[5] =  1;
	dir[6] = -1;  dir[7] = -1; dir[8] = -1;   dir[9] = -1; dir[10] = 1; dir[11] = 1;
	dir[12] = -1; dir[13] = 1; dir[14] = -1;  dir[15] = 1;


	//for thor 3.0
	dir_output[0] = thormang3_kd_->thormang3_link_data[id_r_leg_start + 2*0]->joint_axis.coeff(2, 0);
	dir_output[1] = thormang3_kd_->thormang3_link_data[id_r_leg_start + 2*1]->joint_axis.coeff(0, 0);
	dir_output[2] = thormang3_kd_->thormang3_link_data[id_r_leg_start + 2*2]->joint_axis.coeff(1, 0);
	dir_output[3] = thormang3_kd_->thormang3_link_data[id_r_leg_start + 2*3]->joint_axis.coeff(1, 0);
	dir_output[4] = thormang3_kd_->thormang3_link_data[id_r_leg_start + 2*4]->joint_axis.coeff(1, 0);
	dir_output[5] = thormang3_kd_->thormang3_link_data[id_r_leg_start + 2*5]->joint_axis.coeff(0, 0);


	dir_output[6]  = thormang3_kd_->thormang3_link_data[id_l_leg_start + 2*0]->joint_axis.coeff(2, 0);
	dir_output[7]  = thormang3_kd_->thormang3_link_data[id_l_leg_start + 2*1]->joint_axis.coeff(0, 0);
	dir_output[8]  = thormang3_kd_->thormang3_link_data[id_l_leg_start + 2*2]->joint_axis.coeff(1, 0);
	dir_output[9]  = thormang3_kd_->thormang3_link_data[id_l_leg_start + 2*3]->joint_axis.coeff(1, 0);
	dir_output[10] = thormang3_kd_->thormang3_link_data[id_l_leg_start + 2*4]->joint_axis.coeff(1, 0);
	dir_output[11] = thormang3_kd_->thormang3_link_data[id_l_leg_start + 2*5]->joint_axis.coeff(0, 0);

	dir_output[12] = -1; dir_output[13] = 1;  dir_output[14] = -1; dir_output[15] =  1;


	m_InitAngle[0]  =   0.0;  m_InitAngle[1]  =  0.0;  m_InitAngle[2]  =   0.0;  m_InitAngle[3] =  0.0; m_InitAngle[4]  = 0.0; m_InitAngle[5]  = 0.0;
	m_InitAngle[6]  =   0.0;  m_InitAngle[7]  =  0.0;  m_InitAngle[8]  =   0.0;  m_InitAngle[9] =  0.0; m_InitAngle[10] = 0.0; m_InitAngle[11] = 0.0;
	//m_InitAngle[12] = -45.0,  m_InitAngle[13] = 45.0;  m_InitAngle[14] =  90.0;  m_InitAngle[15] = -90.0;
	m_InitAngle[12] = -45.0,  m_InitAngle[13] = 45.0;  m_InitAngle[14] =  135.0;  m_InitAngle[15] = -135.0;



	r_arm[0] =  -45.0; r_arm[1] =  90.0; r_arm[2] = 0.0;
	r_arm[3] =  135.0; r_arm[4] = -90.0; r_arm[5] = 0.0;

	l_arm[0] =   45.0; l_arm[1] = -90.0; l_arm[2] = 0.0;
	l_arm[3] = -135.0; l_arm[4] =  90.0; l_arm[5] = 0.0;



	for(int i = 0; i < 6; i++) {
		r_arm_init[i] = r_arm[i];
		l_arm_init[i] = l_arm[i];
	}

	m_InitAngle[12] = r_arm[0];  m_InitAngle[13] = l_arm[0];  m_InitAngle[14] = r_arm[3];  m_InitAngle[15] = l_arm[3];


	//////////////////////////////////// Initilaize balancing member variable ////////////////////////////////////
	P_GAIN = 64;
	I_GAIN = 0;
	D_GAIN = 0;



#ifndef WEBOT_SIMULATION
	BALANCE_ENABLE = true;

	COB_X_MANUAL_ADJUSTMENT_M = 0.0;
	COB_Y_MANUAL_ADJUSTMENT_M = 0.0;
	COB_Z_MANUAL_ADJUSTMENT_M = 0.0;

	HIP_ROLL_FEEDFORWARD_ANGLE_RAD = 0;

	DEBUG_PRINT = false;
	HIP_PITCH_OFFSET = 0.0;
	ANKLE_PITCH_OFFSET = 0.0;

	WALK_STABILIZER_GAIN_RATIO = 1.0;
	IMU_GYRO_GAIN_RATIO = 7.31*0.0;
	FORCE_MOMENT_DISTRIBUTION_RATIO = 0.5*0.0;

//	BALANCE_X_GAIN =            +20.30*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
//	BALANCE_Y_GAIN =            -20.30*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
//	BALANCE_Z_GAIN =               2.0*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
//
//	BALANCE_PITCH_GAIN =         -0.06*(1-FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
//	BALANCE_ROLL_GAIN =          -0.10*(1-FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;

	BALANCE_X_GAIN     = +1.0*20.30*0.625*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
	BALANCE_Y_GAIN     = -1.0*20.30*0.75*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
	BALANCE_Z_GAIN     =       0.0*2.0*(FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;

	BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;
	BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - FORCE_MOMENT_DISTRIBUTION_RATIO)*WALK_STABILIZER_GAIN_RATIO;



	/// Damping
	BALANCE_HIP_ROLL_GAIN = 0.0;
	BALANCE_HIP_PITCH_GAIN = 0.0;

	BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = -0.5;
	BALANCE_ANKLE_PITCH_GAIN_BY_IMU = -0.5;

	BALANCE_X_GAIN_BY_FT = 0.125;
	BALANCE_Y_GAIN_BY_FT = 0.125;
	BALANCE_Z_GAIN_BY_FT = -0.1*0.5;

	BALANCE_RIGHT_ROLL_GAIN_BY_FT = 0.01*0.1;
	BALANCE_RIGHT_PITCH_GAIN_BY_FT = 0.01*0.1*0.5;

	BALANCE_LEFT_ROLL_GAIN_BY_FT = 0.01*0.1;
	BALANCE_LEFT_PITCH_GAIN_BY_FT = 0.01*0.1*0.5;


	BALANCE_HIP_ROLL_TIME_CONSTANT = 1.0;
	BALANCE_HIP_PITCH_TIME_CONSTANT = 1.0;



	BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU	 = 0.2;
	BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU = 0.2;

	BALANCE_X_TIME_CONSTANT					 = 0.1;
	BALANCE_Y_TIME_CONSTANT					 = 0.1;
	BALANCE_Z_TIME_CONSTANT					 = 0.1;
	BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT	 = 0.1;
	BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT	 = 0.1;
	BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT	 = 0.1;
	BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT	 = 0.1;




	AXIS_CONTROLLER_GAIN = 0.0;
	AXIS_CONTROLLER_TIME_CONSTANT = 1.0*0.01;

	LANDING_CONTROLLER_GAIN = 0.0;
	LANDING_CONTROLLER_TIME_CONSTANT = 0.5*0.2;
	/// Damping

	FOOT_LANDING_OFFSET_GAIN =   +1.0*0;
	FOOT_LANDING_DETECT_N =   50;

	if(TIME_UNIT < 1.0)
		SYSTEM_CONTROL_UNIT_TIME_SEC = TIME_UNIT;
	else
		SYSTEM_CONTROL_UNIT_TIME_SEC = TIME_UNIT / 1000.0;

	FOOT_LANDING_DETECTION_TIME_MAX_SEC = 1.0;

	FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD = 10.0*M_PI/180;
	FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD = 10.0*M_PI/180;

	FOOT_ROLL_ADJUSTMENT_ABS_MAX_RAD_BY_FT  = 5.0*M_PI/180.0;
	FOOT_PITCH_ADJUSTMENT_ABS_MAX_RAD_BY_FT = 5.0*M_PI/180.0;

	COB_X_ADJUSTMENT_ABS_MAX_MM = 50;
	COB_Y_ADJUSTMENT_ABS_MAX_MM = 50;
	COB_Z_ADJUSTMENT_ABS_MAX_MM = 50;



	X_ADJUSTMENT_ABS_MAX_MM 	 = 50.0;
	Y_ADJUSTMENT_ABS_MAX_MM 	 = 50.0;
	Z_ADJUSTMENT_ABS_MAX_MM 	 = 50.0;
	ROLL_ADJUSTMENT_ABS_MAX_RAD  = 15.0*M_PI/180.0;
	PITCH_ADJUSTMENT_ABS_MAX_RAD = 15.0*M_PI/180.0;


	current_right_fx_N  = current_right_fy_N  = current_right_fz_N  = 0;
	current_right_Tx_Nm = current_right_Ty_Nm = current_right_Tz_Nm = 0;
	current_left_fx_N  = current_left_fy_N  = current_left_fz_N  = 0;
	current_left_Tx_Nm = current_left_Ty_Nm = current_left_Tz_Nm = 0;

	current_imu_roll_rad = current_imu_pitch_rad = 0;
	current_gyro_roll_rad_per_sec = current_gyro_pitch_rad_per_sec = 0;


	foot_landing_detection_time_sec = 0;

	foot_r_roll_landing_offset_rad = 0;
	foot_r_pitch_landing_offset_rad = 0;
	foot_l_roll_landing_offset_rad = 0;
	foot_l_pitch_landing_offset_rad = 0;

	foot_r_roll_adjustment_rad = 0;
	foot_r_pitch_adjustment_rad = 0;
	foot_l_roll_adjustment_rad = 0;
	foot_l_pitch_adjustment_rad = 0;


	cob_x_adjustment_mm = 0;
	cob_y_adjustment_mm = 0;
	cob_z_adjustment_mm = 0;

	m_gyro_roll_rad_per_sec  = 0;
	m_gyro_pitch_rad_per_sec = 0;

	/// Damping
	hip_roll_adjustment_deg = 0;
	hip_pitch_adjustment_deg = 0;
	r_x_adjustment_mm_by_ft = r_y_adjustment_mm_by_ft = 0;
	l_x_adjustment_mm_by_ft = l_y_adjustment_mm_by_ft = 0;
	z_adjustment_mm_by_ft = 0;

	foot_r_roll_adjustment_rad_by_ft  = 0;
	foot_r_pitch_adjustment_rad_by_ft = 0;
	foot_l_roll_adjustment_rad_by_ft  = 0;
	foot_l_pitch_adjustment_rad_by_ft = 0;

	foot_r_roll_adjustment_rad_by_imu  = 0;
	foot_r_pitch_adjustment_rad_by_imu = 0;
	foot_l_roll_adjustment_rad_by_imu  = 0;
	foot_l_pitch_adjustment_rad_by_imu = 0;


	cob_x_adjustment_mm_by_landing_controller = 0;
	cob_y_adjustment_mm_by_landing_controller = 0;

	/// Damping

	m_init_right_fx_N = 0,  m_init_right_fy_N = 0,  m_init_right_fz_N = 0;
	m_init_right_Tx_Nm = 0, m_init_right_Ty_Nm = 0, m_init_right_Tz_Nm = 0;
	m_init_left_fx_N = 0,  m_init_left_fy_N = 0,  m_init_left_fz_N = 0;
	m_init_left_Tx_Nm = 0, m_init_left_Ty_Nm = 0, m_init_left_Tz_Nm = 0;

	m_right_ft_scale_factor = 1.0;
	m_left_ft_scale_factor = 1.0;


	double _total_mass_of_robot = thormang3_kd_->TotalMass(0);
//	printf("ROBOT TOTAL MASS : %f \n", _total_mass_of_robot);
//	m_right_dsp_fz_N = -1.0*(42.0)*9.8*0.5;
//	m_right_ssp_fz_N = -1.0*(42.0)*9.8;
//	m_left_dsp_fz_N  = -1.0*(42.0)*9.8*0.5;
//	m_left_ssp_fz_N  = -1.0*(42.0)*9.8;

	m_right_dsp_fz_N = -1.0*(_total_mass_of_robot)*9.8*0.5;
	m_right_ssp_fz_N = -1.0*(_total_mass_of_robot)*9.8;
	m_left_dsp_fz_N  = -1.0*(_total_mass_of_robot)*9.8*0.5;
	m_left_ssp_fz_N  = -1.0*(_total_mass_of_robot)*9.8;

	m_Left_Fz_Sigmoid_StartTime = 0;
	m_Left_Fz_Sigmoid_EndTime  = 0;
	m_Left_Fz_Sigmoid_Target  = m_left_dsp_fz_N;
	m_Left_Fz_Sigmoid_Shift   = m_left_dsp_fz_N;

	gyro_roll_init_rad_per_sec = 0;
	gyro_pitch_init_rad_per_sec = 0;
	gyro_yaw_init_rad_per_sec = 0;

	m_iu_roll_init_rad = 0;
	m_iu_pitch_init_rad = 0;
	m_iu_yaw_init_rad = 0;
#endif

	pthread_mutex_init(&m_mutex_lock, NULL);

	m_CurrentStartIdxforZMPRef = 0;
	zmp_ref_x_at_this_time = 0;
	zmp_ref_y_at_this_time = 0;
	sum_of_zmp_x = 0;
	sum_of_zmp_y = 0;
	sum_of_cx = 0;
	sum_of_cy = 0;

}

PreviewControlWalking::~PreviewControlWalking()
{
//	fclose(fpBalance);
//	fclose(fpPreviewControlWalking);
	//fclose(fpAxisController);
}

bool PreviewControlWalking::SetInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
		double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
		double center_of_body_x, double center_of_body_y, double center_of_body_z,
		double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw)
{

	if(m_Real_Running || m_Ctrl_Running)
		return false;


	m_PreviousStepRightFootPosition.x     = r_foot_x;
	m_PreviousStepRightFootPosition.y     = r_foot_y;
	m_PreviousStepRightFootPosition.z     = r_foot_z;
	m_PreviousStepRightFootPosition.roll  = r_foot_roll;
	m_PreviousStepRightFootPosition.pitch = r_foot_pitch;
	m_PreviousStepRightFootPosition.yaw   = r_foot_yaw;


	m_PreviousStepLeftFootPosition.x     = l_foot_x;
	m_PreviousStepLeftFootPosition.y     = l_foot_y;
	m_PreviousStepLeftFootPosition.z     = l_foot_z;
	m_PreviousStepLeftFootPosition.roll  = l_foot_roll;
	m_PreviousStepLeftFootPosition.pitch = l_foot_pitch;
	m_PreviousStepLeftFootPosition.yaw   = l_foot_yaw;

	m_PreviousStepBodyPosition.x     = center_of_body_x;
	m_PreviousStepBodyPosition.y     = center_of_body_y;
	m_PreviousStepBodyPosition.z     = center_of_body_z;
	m_PreviousStepBodyPosition.roll  = center_of_body_roll;
	m_PreviousStepBodyPosition.pitch = center_of_body_pitch;
	m_PreviousStepBodyPosition.yaw   = center_of_body_yaw;


	m_InitialRightFootPosition = m_PreviousStepRightFootPosition;
	m_InitialLeftFootPosition  = m_PreviousStepLeftFootPosition;
	m_InitialBodyPosition      = m_PreviousStepBodyPosition;

	//These matrix and parameters are for preview control
	K_s_ = 0.0;
	sum_of_zmp_x = 0.0;
	sum_of_zmp_y = 0.0;
	sum_of_cx = 0.0;
	sum_of_cy = 0.0;

	return true;
}

void PreviewControlWalking::SetInitalWaistYawAngle(double waist_yaw_angle_rad)
{
	m_GoalWaistYawAngleRad = waist_yaw_angle_rad;
}



void PreviewControlWalking::Initialize()
{
	if(m_Real_Running)
		return;



	pthread_mutex_lock(&m_mutex_lock);
	m_StepData.clear();


	//Initialize Time
	m_WalkingTime = 0; m_ReferenceTime = 0;

	//Initializing m_ZMP_X Init
	matd matRotationGtoROrigin, inv_matRotationGtoOrigin;
	matd inv_matTranslationGtoROrigin;
	double g_to_robot_origin_x, g_to_robot_origin_y, g_to_robot_origin_z;

	matRotationGtoROrigin.resize(4,4);
	matRotationGtoROrigin = GetOrientationMatrix(0.0, 0.0, m_PresentBodyPosition.yaw);

	inv_matRotationGtoOrigin = matRotationGtoROrigin.transpose();

	g_to_robot_origin_x = 0.5*(m_PreviousStepRightFootPosition.x + m_PreviousStepLeftFootPosition.x);
	g_to_robot_origin_y = 0.5*(m_PreviousStepRightFootPosition.y + m_PreviousStepLeftFootPosition.y);
	g_to_robot_origin_z = 0.5*(m_PreviousStepRightFootPosition.z + m_PreviousStepLeftFootPosition.z);

	inv_matTranslationGtoROrigin = GetTranslationMatrix(-g_to_robot_origin_x, -g_to_robot_origin_y, -g_to_robot_origin_z);



	m_PreviousStepLeftFootPosition = GetPose3DfromTransformMatrix(inv_matRotationGtoOrigin*inv_matTranslationGtoROrigin*GetTransformMatrix(m_PreviousStepLeftFootPosition.x, m_PreviousStepLeftFootPosition.y, m_PreviousStepLeftFootPosition.z,
			m_PreviousStepLeftFootPosition.roll, m_PreviousStepLeftFootPosition.pitch, m_PreviousStepLeftFootPosition.yaw));

	m_PreviousStepRightFootPosition = GetPose3DfromTransformMatrix(inv_matRotationGtoOrigin*inv_matTranslationGtoROrigin*GetTransformMatrix(m_PreviousStepRightFootPosition.x, m_PreviousStepRightFootPosition.y, m_PreviousStepRightFootPosition.z,
			m_PreviousStepRightFootPosition.roll, m_PreviousStepRightFootPosition.pitch, m_PreviousStepRightFootPosition.yaw));

	m_PreviousStepBodyPosition = GetPose3DfromTransformMatrix(inv_matRotationGtoOrigin*inv_matTranslationGtoROrigin*GetTransformMatrix(m_PreviousStepBodyPosition.x, m_PreviousStepBodyPosition.y, m_PreviousStepBodyPosition.z,
			m_PreviousStepBodyPosition.roll, m_PreviousStepBodyPosition.pitch, m_PreviousStepBodyPosition.yaw));


	m_PresentRightFootPosition = m_PreviousStepRightFootPosition;
	m_PresentLeftFootPosition  = m_PreviousStepLeftFootPosition;
	m_PresentBodyPosition      = m_PreviousStepBodyPosition;

	matd CurrentCOMList;
	matGtoCOB = GetTransformMatrix(m_PreviousStepBodyPosition.x, m_PreviousStepBodyPosition.y, m_PreviousStepBodyPosition.z,
			m_PreviousStepBodyPosition.roll, m_PreviousStepBodyPosition.pitch, m_PreviousStepBodyPosition.yaw);

	matCOBtoG = GetTransformMatrixInverse(matGtoCOB);

	matGtoRF = GetTransformMatrix(m_PreviousStepRightFootPosition.x, m_PreviousStepRightFootPosition.y, m_PreviousStepRightFootPosition.z,
			m_PreviousStepRightFootPosition.roll, m_PreviousStepRightFootPosition.pitch, m_PreviousStepRightFootPosition.yaw);
	matGtoLF = GetTransformMatrix(m_PreviousStepLeftFootPosition.x, m_PreviousStepLeftFootPosition.y, m_PreviousStepLeftFootPosition.z,
			m_PreviousStepLeftFootPosition.roll, m_PreviousStepLeftFootPosition.pitch, m_PreviousStepLeftFootPosition.yaw);

	matRHtoRF = matRHtoCOB*matCOBtoG*matGtoRF;
	matLHtoLF = matLHtoCOB*matCOBtoG*matGtoLF;

	Pose3D epr = GetPose3DfromTransformMatrix(matRHtoRF);
	Pose3D epl = GetPose3DfromTransformMatrix(matLHtoLF);


	epr.x -=  COB_X_MANUAL_ADJUSTMENT_M;
	epl.x -=  COB_X_MANUAL_ADJUSTMENT_M;
	double angle[12];
	if(thormang3_kd_->InverseKinematicsforRightLeg(&angle[0], epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw) == false) {
		printf("IK not Solved EPR : %f %f %f %f %f %f\n", epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw);
		return;
	}

	if(thormang3_kd_->InverseKinematicsforLeftLeg(&angle[6], epl.x, epl.y, epl.z, epl.roll, epl.pitch, epl.yaw) == false) {
		printf("IK not Solved EPL : %f %f %f %f %f %f\n", epl.x, epr.y, epl.z, epl.roll, epl.pitch, epl.yaw);
		return;
	}


	double rangle[6], langle[6];
	for(int idx = 0; idx < 6; idx++)
	{
		rangle[idx] = angle[idx]*dir[idx]*180.0/M_PI;
		langle[idx] = angle[idx+6]*dir[idx+6]*180.0/M_PI;
	}



	m_ReferenceStepDataforAddition.PositionData.bMovingFoot = NFootMove;
	m_ReferenceStepDataforAddition.PositionData.dElbowSwingGain = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dShoulderSwingGain = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dFootHeight = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dWaistPitchAngle = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dWaistYawAngle = m_GoalWaistYawAngleRad;
	m_ReferenceStepDataforAddition.PositionData.dZ_Swap_Amplitude = 0.0;
	m_ReferenceStepDataforAddition.PositionData.stBodyPosition = m_PreviousStepBodyPosition;
	m_ReferenceStepDataforAddition.PositionData.stRightFootPosition = m_PreviousStepRightFootPosition;
	m_ReferenceStepDataforAddition.PositionData.stLeftFootPosition = m_PreviousStepLeftFootPosition;
	m_ReferenceStepDataforAddition.TimeData.bWalkingState = InWalkingEnding;
	m_ReferenceStepDataforAddition.TimeData.dAbsStepTime = 0.0;
	m_ReferenceStepDataforAddition.TimeData.dDSPratio = 0.2;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_yaw = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_yaw = 1.0;

	m_PresentWaistYawAngleRad = m_GoalWaistYawAngleRad;
	m_PreviousStepWaistYawAngleRad = m_GoalWaistYawAngleRad;

	m_CurrentStepDataStatus = StepDataStatus4;

	//Initialize Matrix for Preview Control
	double t = 0;
	if(TIME_UNIT < 1.0)
		t=TIME_UNIT;
	else
		t=TIME_UNIT/1000.0;

	//double z_COM_mm = m_PreviousStepBodyPosition.z;
	A.resize(3,3); b.resize(3,1); c.resize(1,3);
	A << 1,  t, t*t/2.0,
			0,  1,   t,
			0,  0,   1;
	b(0,0) = t*t*t/6.0;
	b(1,0) =   t*t/2.0;
	b(2,0) =	 t;


	c(0,0) = 1; c(0,1) = 0; c(0,2) = -500.0*MMtoM/G;

	double Q_e = 1.0, R = 0.000001;//1.0e-6;
	m_PreviewSize = round(m_PreviewTime/TIME_UNIT);


	m_StepIdxData.resize(m_PreviewSize);
	m_StepIdxData.fill(NO_STEP_IDX);
	m_CurrentStartIdxforZMPRef = 0;
	m_ZMP_Reference_X.resize(m_PreviewSize, 1);
	m_ZMP_Reference_X.fill(0.5*(m_PresentRightFootPosition.x + m_PresentLeftFootPosition.x));
	m_ZMP_Reference_Y.resize(m_PreviewSize, 1);
	m_ZMP_Reference_Y.fill(0.5*(m_PresentRightFootPosition.y + m_PresentLeftFootPosition.y));




	K_s_ =608.900142915471; //500.0

	K_x_.resize(1,3);
	K_x_ << 35904.1790662895, 8609.63092261379, 112.710622775482; // 500

	f_.resize(1, m_PreviewSize);
	//500
	f_ << 608.9001429,  760.1988656,  921.629618,  1034.769891,   1089.313783,   1096.765451,  1074.295061,  1036.842257,   994.5569472,  953.0295724,
			914.5708045,  879.5752596,  847.5633512,  817.8237132,   789.7293917,   762.8419074,  736.9038288,  711.7889296,  687.4484508,  663.8698037,
			641.050971,   618.987811,   597.6697941,  577.0801943,   557.1979969,   537.9999833,  519.4623366,  501.5616312,  484.2753137,  467.581849,
			451.4606896,  435.8921765,  420.8574331,  406.3382777,   392.3171623,   378.7771307,  365.7017923,  353.0753019,  340.8823445,  329.1081203,
			317.7383294,  306.759157,   296.1572572,  285.919738,    276.0341458,   266.4884502,  257.2710302,  248.3706597,  239.7764944,  231.4780587,
			223.4652335,  215.7282439,  208.2576474,  201.0443231,   194.0794606,   187.3545495,  180.8613691,  174.5919788,  168.5387087,  162.6941501,
			157.0511468,  151.6027868,  146.3423936,  141.2635185,   136.3599326,   131.62562,    127.0547695,  122.6417688,  118.3811969,  114.2678179,
			110.2965751,  106.462584,   102.7611275,   99.18764938,   95.73774938,   92.40717757,  89.19182939,  86.08774068,  83.0910829,   80.19815849,
			77.40539648,  74.70934812,  72.10668274,  69.59418373,   67.16874468,   64.82736558,  62.56714924,  60.38529775,  58.27910913,  56.24597404,
			54.28337262,  52.38887145,  50.5601206,   48.79485075,   47.09087052,   45.44606371,  43.8583868,   42.32586646,  40.84659712,  39.4187387,
			38.04051434,  36.71020825,  35.42616363,  34.18678064,   32.99051448,   31.83587345,  30.72141721,  29.64575495,  28.60754377,  27.60548695,
			26.63833247,  25.70487138,  24.80393641,  23.93440049,   23.09517539,   22.28521038,  21.50349096,  20.74903761,  20.0209046,   19.3181788,
			18.63997859,  17.98545279,  17.35377958,  16.74416551,   16.15584454,   15.58807707,  15.04014906,  14.51137112,  14.00107771,  13.50862627,
			13.03339646,  12.57478939,  12.13222688,  11.70515076,   11.29302214,   10.89532081,  10.51154456,  10.14120854,   9.783844714,  9.439001248,
			9.106241955,  8.78514576,   8.475306179,  8.176330819,   7.887840885,   7.609470721,  7.340867346,  7.081690027,  6.83160985,   6.590309314,
			6.357481938,  6.132831881,  5.916073573,  5.706931359,   5.505139163,   5.310440149,  5.122586408,  4.941338646,  4.766465888,  4.597745188,
			4.434961356,  4.277906685,  4.126380694,  3.980189879,   3.839147471,   3.703073201,  3.571793078,  3.445139169,  3.322949391,  3.205067309,
			3.091341936,  2.981627549,  2.875783507,  2.773674068,   2.675168228,   2.58013955,   2.488466009,  2.400029838,  2.314717381,  2.232418947,
			2.153028678,  2.076444411,  2.002567552,  1.931302952,   1.862558786,   1.796246438,  1.732280393,  1.670578121,  1.611059983,  1.553649123,
			1.498271374,  1.444855165,  1.393331431,  1.343633522,   1.295697125,   1.249460178,  1.204862791,  1.161847176,  1.120357568,  1.080340156;


	u_x.resize(1,1);
	u_y.resize(1,1);

	//std::cout << f_Preview << std::endl;

	x_LIPM.resize(3, 1);	y_LIPM.resize(3, 1);
	x_LIPM.fill(0.0);		y_LIPM.fill(0.0);



	pthread_mutex_unlock(&m_mutex_lock);

	for(int idx = 0; idx < 6; idx++)
	{
		m_OutAngleRad[idx]   = angle[idx];
		m_OutAngleRad[idx+6] = angle[idx+6];
	}


	printf("RLEG : %f %f %f %f %f %f\n", m_OutAngleRad[0]*180.0/M_PI, m_OutAngleRad[1]*180.0/M_PI, m_OutAngleRad[2]*180.0/M_PI,
			m_OutAngleRad[3]*180.0/M_PI, m_OutAngleRad[4]*180.0/M_PI, m_OutAngleRad[5]*180.0/M_PI);
	printf("LLEG : %f %f %f %f %f %f\n", m_OutAngleRad[6]*180.0/M_PI, m_OutAngleRad[7]*180.0/M_PI, m_OutAngleRad[8]*180.0/M_PI,
			m_OutAngleRad[9]*180.0/M_PI, m_OutAngleRad[10]*180.0/M_PI, m_OutAngleRad[11]*180.0/M_PI);

	m_OutAngleRad[12] = ((double)dir_output[12]*(epr.x - epl.x)*m_ShoulerSwingGain + m_InitAngle[12])*M_PI/180.0;
	m_OutAngleRad[13] = ((double)dir_output[13]*(epl.x - epr.x)*m_ShoulerSwingGain + m_InitAngle[13])*M_PI/180.0;
	m_OutAngleRad[14] = ((double)dir_output[14]*(epr.x - epl.x)*m_ElbowSwingGain   + m_InitAngle[14])*M_PI/180.0;
	m_OutAngleRad[15] = ((double)dir_output[15]*(epl.x - epr.x)*m_ElbowSwingGain   + m_InitAngle[15])*M_PI/180.0;


	m_OutAngleRad[2] -= (double)dir_output[2] * (HIP_PITCH_OFFSET) * (M_PI)/180.0;
	m_OutAngleRad[8] -= (double)dir_output[8] * (HIP_PITCH_OFFSET) * (M_PI)/180.0;

	m_OutAngleRad[4]  += (double)dir_output[4]  * ANKLE_PITCH_OFFSET *(M_PI)/180.0;
	m_OutAngleRad[10] += (double)dir_output[10] * ANKLE_PITCH_OFFSET *(M_PI)/180.0;


	m_Left_Fz_Sigmoid_StartTime = 0;
	m_Left_Fz_Sigmoid_EndTime  = 0;
	m_Left_Fz_Sigmoid_Target  = m_left_dsp_fz_N;
	m_Left_Fz_Sigmoid_Shift   = m_left_dsp_fz_N;

}
//
void PreviewControlWalking::LocalizeAllWalkingParameter()
{
	if(m_Real_Running)
		return;



	pthread_mutex_lock(&m_mutex_lock);
	m_StepData.clear();


	//Initialize Time
	m_WalkingTime = 0; m_ReferenceTime = 0;

	//Initializing m_ZMP_X Init
	matd matRotationGtoROrigin, inv_matRotationGtoOrigin;
	matd inv_matTranslationGtoROrigin;
	double g_to_robot_origin_x, g_to_robot_origin_y, g_to_robot_origin_z;

//	matRotationGtoROrigin.resize(4,4);
//	matRotationGtoROrigin = GetOrientationMatrix(0.0, 0.0, m_PresentBodyPosition.yaw);
//
//	inv_matRotationGtoOrigin = matRotationGtoROrigin.transpose();

	m_PreviousStepRightFootPosition = m_InitialRightFootPosition;
	m_PreviousStepLeftFootPosition = m_InitialLeftFootPosition;
	m_PreviousStepBodyPosition = m_InitialBodyPosition;


//	g_to_robot_origin_x = 0.5*(m_PreviousStepRightFootPosition.x + m_PreviousStepLeftFootPosition.x);
//	g_to_robot_origin_y = 0.5*(m_PreviousStepRightFootPosition.y + m_PreviousStepLeftFootPosition.y);
//	g_to_robot_origin_z = 0.5*(m_PreviousStepRightFootPosition.z + m_PreviousStepLeftFootPosition.z);
//
//	inv_matTranslationGtoROrigin = GetTranslationMatrix(-g_to_robot_origin_x, -g_to_robot_origin_y, -g_to_robot_origin_z);
//
//
//
//	m_PreviousStepLeftFootPosition = GetPose3DfromTransformMatrix(inv_matRotationGtoOrigin*inv_matTranslationGtoROrigin*GetTransformMatrix(m_PreviousStepLeftFootPosition.x, m_PreviousStepLeftFootPosition.y, m_PreviousStepLeftFootPosition.z,
//			m_PreviousStepLeftFootPosition.roll, m_PreviousStepLeftFootPosition.pitch, m_PreviousStepLeftFootPosition.yaw));
//
//	m_PreviousStepRightFootPosition = GetPose3DfromTransformMatrix(inv_matRotationGtoOrigin*inv_matTranslationGtoROrigin*GetTransformMatrix(m_PreviousStepRightFootPosition.x, m_PreviousStepRightFootPosition.y, m_PreviousStepRightFootPosition.z,
//			m_PreviousStepRightFootPosition.roll, m_PreviousStepRightFootPosition.pitch, m_PreviousStepRightFootPosition.yaw));
//
//	m_PreviousStepBodyPosition = GetPose3DfromTransformMatrix(inv_matRotationGtoOrigin*inv_matTranslationGtoROrigin*GetTransformMatrix(m_PreviousStepBodyPosition.x, m_PreviousStepBodyPosition.y, m_PreviousStepBodyPosition.z,
//			m_PreviousStepBodyPosition.roll, m_PreviousStepBodyPosition.pitch, m_PreviousStepBodyPosition.yaw));


	m_PresentRightFootPosition = m_PreviousStepRightFootPosition;
	m_PresentLeftFootPosition  = m_PreviousStepLeftFootPosition;
	m_PresentBodyPosition      = m_PreviousStepBodyPosition;

	matd CurrentCOMList;
	matGtoCOB = GetTransformMatrix(m_PreviousStepBodyPosition.x, m_PreviousStepBodyPosition.y, m_PreviousStepBodyPosition.z,
			m_PreviousStepBodyPosition.roll, m_PreviousStepBodyPosition.pitch, m_PreviousStepBodyPosition.yaw);

	matCOBtoG = GetTransformMatrixInverse(matGtoCOB);

	matGtoRF = GetTransformMatrix(m_PreviousStepRightFootPosition.x, m_PreviousStepRightFootPosition.y, m_PreviousStepRightFootPosition.z,
			m_PreviousStepRightFootPosition.roll, m_PreviousStepRightFootPosition.pitch, m_PreviousStepRightFootPosition.yaw);
	matGtoLF = GetTransformMatrix(m_PreviousStepLeftFootPosition.x, m_PreviousStepLeftFootPosition.y, m_PreviousStepLeftFootPosition.z,
			m_PreviousStepLeftFootPosition.roll, m_PreviousStepLeftFootPosition.pitch, m_PreviousStepLeftFootPosition.yaw);

	matRHtoRF = matRHtoCOB*matCOBtoG*matGtoRF;
	matLHtoLF = matLHtoCOB*matCOBtoG*matGtoLF;

	Pose3D epr = GetPose3DfromTransformMatrix(matRHtoRF);
	Pose3D epl = GetPose3DfromTransformMatrix(matLHtoLF);


	epr.x -=  COB_X_MANUAL_ADJUSTMENT_M;
	epl.x -=  COB_X_MANUAL_ADJUSTMENT_M;


	double angle[12];
	if(thormang3_kd_->InverseKinematicsforRightLeg(&angle[0], epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw) == false)	{
		printf("IK not Solved EPR : %f %f %f %f %f %f\n", epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw);
		return;
	}

	if(thormang3_kd_->InverseKinematicsforLeftLeg(&angle[6], epl.x, epl.y, epl.z, epl.roll, epl.pitch, epl.yaw) == false)	{
		printf("IK not Solved EPL : %f %f %f %f %f %f\n", epl.x, epr.y, epl.z, epl.roll, epl.pitch, epl.yaw);
		return;
	}


	double rangle[6], langle[6];
	for(int idx = 0; idx < 6; idx++)
	{
		rangle[idx] = angle[idx]*dir[idx]*180.0/M_PI;
		langle[idx] = angle[idx+6]*dir[idx+6]*180.0/M_PI;
	}

	m_ReferenceStepDataforAddition.PositionData.bMovingFoot = NFootMove;
	m_ReferenceStepDataforAddition.PositionData.dElbowSwingGain = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dShoulderSwingGain = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dFootHeight = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dWaistPitchAngle = 0.0;
	m_ReferenceStepDataforAddition.PositionData.dWaistYawAngle = m_GoalWaistYawAngleRad;
	m_ReferenceStepDataforAddition.PositionData.dZ_Swap_Amplitude = 0.0;
	m_ReferenceStepDataforAddition.PositionData.stBodyPosition = m_PreviousStepBodyPosition;
	m_ReferenceStepDataforAddition.PositionData.stRightFootPosition = m_PreviousStepRightFootPosition;
	m_ReferenceStepDataforAddition.PositionData.stLeftFootPosition = m_PreviousStepLeftFootPosition;
	m_ReferenceStepDataforAddition.TimeData.bWalkingState = InWalkingEnding;
	m_ReferenceStepDataforAddition.TimeData.dAbsStepTime = 0.0;
	m_ReferenceStepDataforAddition.TimeData.dDSPratio = 0.2;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_yaw = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_yaw = 1.0;

	m_PresentWaistYawAngleRad = m_GoalWaistYawAngleRad;
	m_PreviousStepWaistYawAngleRad = m_GoalWaistYawAngleRad;

	m_CurrentStepDataStatus = StepDataStatus4;




	m_StepIdxData.fill(NO_STEP_IDX);
	m_CurrentStartIdxforZMPRef = 0;
	m_ZMP_Reference_X.fill(0.5*(m_PresentRightFootPosition.x + m_PresentLeftFootPosition.x));
	m_ZMP_Reference_Y.fill(0.5*(m_PresentRightFootPosition.y + m_PresentLeftFootPosition.y));
	sum_of_zmp_x = 0.0;
	sum_of_zmp_y = 0.0;

	sum_of_cx = 0.0;
	sum_of_cy = 0.0;
	x_LIPM.fill(0.0);		y_LIPM.fill(0.0);



	pthread_mutex_unlock(&m_mutex_lock);

	for(int idx = 0; idx < 6; idx++)
	{
		m_OutAngleRad[idx]   = angle[idx];
		m_OutAngleRad[idx+6] = angle[idx+6];
	}

	m_OutAngleRad[12] = ((double)dir_output[12]*(epr.x - epl.x)*m_ShoulerSwingGain + m_InitAngle[12])*M_PI/180.0;
	m_OutAngleRad[13] = ((double)dir_output[13]*(epl.x - epr.x)*m_ShoulerSwingGain + m_InitAngle[13])*M_PI/180.0;
	m_OutAngleRad[14] = ((double)dir_output[14]*(epr.x - epl.x)*m_ElbowSwingGain   + m_InitAngle[14])*M_PI/180.0;
	m_OutAngleRad[15] = ((double)dir_output[15]*(epl.x - epr.x)*m_ElbowSwingGain   + m_InitAngle[15])*M_PI/180.0;


	m_OutAngleRad[2] -= (double)dir_output[2] * (HIP_PITCH_OFFSET) * (M_PI)/180.0;
	m_OutAngleRad[8] -= (double)dir_output[8] * (HIP_PITCH_OFFSET) * (M_PI)/180.0;

	m_OutAngleRad[4]  += (double)dir_output[4]  * ANKLE_PITCH_OFFSET *(M_PI)/180.0;
	m_OutAngleRad[10] += (double)dir_output[10] * ANKLE_PITCH_OFFSET *(M_PI)/180.0;


	m_Left_Fz_Sigmoid_StartTime = 0;
	m_Left_Fz_Sigmoid_EndTime  = 0;
	m_Left_Fz_Sigmoid_Target  = m_left_dsp_fz_N;
	m_Left_Fz_Sigmoid_Shift   = m_left_dsp_fz_N;

}
//
void PreviewControlWalking::Start()
{
	m_Ctrl_Running = true;
	m_Real_Running = true;
}

void PreviewControlWalking::Stop()
{
	m_Ctrl_Running = false;
}

bool PreviewControlWalking::IsRunning()
{
	return m_Real_Running;
}

bool PreviewControlWalking::AddStepData(StepData step_data)
{


	if(m_CurrentStepDataStatus == StepDataStatus3 || m_CurrentStepDataStatus == StepDataStatus4 )
	{
		if(GetNumofRemainingUnreservedStepData() == 0) {
			pthread_mutex_lock(&m_mutex_lock);
			//m_StepData.push_back(m_ReferenceStepDataforAddition);
			pthread_mutex_unlock(&m_mutex_lock);
		}
	}

	pthread_mutex_lock(&m_mutex_lock);
	m_StepData.push_back(step_data);

	CalcStepIdxData();
	pthread_mutex_unlock(&m_mutex_lock);

	return true;
}

int PreviewControlWalking::GetNumofRemainingUnreservedStepData()
{
	int step_idx = m_StepIdxData(m_PreviewSize - 1);
	int remain_step_num = 0;
	if(step_idx != NO_STEP_IDX)
	{
		remain_step_num = (m_StepData.size() - 1 - step_idx);
	}
	else {
		remain_step_num = 0;
	}
	return remain_step_num;
}

void PreviewControlWalking::EraseLastStepData()
{
	pthread_mutex_lock(&m_mutex_lock);
	if(GetNumofRemainingUnreservedStepData() != 0)
	{
		m_StepData.pop_back();
	}
	pthread_mutex_unlock(&m_mutex_lock);
}

void PreviewControlWalking::GetReferenceStepDatafotAddition(StepData *ref_step_data_for_addition)
{
	//	if(m_CurrentStepDataStatus == StepDataStatus1) {
	//		int step_idx = m_StepIdxData(m_StepIdxData.size() - 1);
	//		m_ReferenceStepDataforAddition = m_StepData[step_idx];;
	//	}
	//	else if(m_CurrentStepDataStatus == StepDataStatus2) {
	//		m_ReferenceStepDataforAddition = m_StepIdxData[m_StepData.end()];
	//	}
	//	else if(m_CurrentStepDataStatus == StepDataStatus3) {
	//		m_ReferenceStepDataforAddition = m_StepIdxData[m_StepData.end()];
	//		m_ReferenceStepDataforAddition.TimeData += m_PreviewTime;
	//	}
	//	else if(m_CurrentStepDataStatus == StepDataStatus4) {
	//		m_ReferenceStepDataforAddition.TimeData += m_WalkingTime + m_PresentBodyPosition;
	//	}
	//	else
	//		return;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_ratio_yaw = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_x = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_y = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_z = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_roll = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_pitch = 1.0;
	m_ReferenceStepDataforAddition.TimeData.sigmoid_distortion_yaw = 1.0;
	(*ref_step_data_for_addition) = m_ReferenceStepDataforAddition;
}

void PreviewControlWalking::CalcStepIdxData()
{
	unsigned int step_idx = 0, previous_step_idx = 0;
	unsigned int step_data_size = m_StepData.size();
	if(m_StepData.size() == 0) {
		m_StepIdxData.fill(NO_STEP_IDX);
		m_CurrentStepDataStatus = StepDataStatus4;
		m_Real_Running = false;
		//return;
	}
	else {
		if(m_WalkingTime >= m_StepData[0].TimeData.dAbsStepTime - 0.5*MStoS) {
			m_PreviousStepWaistYawAngleRad = m_StepData[0].PositionData.dWaistYawAngle;
			m_PreviousStepLeftFootPosition = m_StepData[0].PositionData.stLeftFootPosition;
			m_PreviousStepRightFootPosition = m_StepData[0].PositionData.stRightFootPosition;
			m_PreviousStepBodyPosition = m_StepData[0].PositionData.stBodyPosition;
			m_PreviousStepBodyPosition.x = m_PresentBodyPosition.x;
			m_PreviousStepBodyPosition.y = m_PresentBodyPosition.y;
			m_ReferenceTime = m_StepData[0].TimeData.dAbsStepTime;
			m_StepData.erase(m_StepData.begin());
			if(m_StepData.size() == 0) {
				m_StepIdxData.fill(NO_STEP_IDX);
				m_CurrentStepDataStatus = StepDataStatus4;
				m_Real_Running = false;
				//return;
			}
			else {
				for(int idx = 0; idx < m_PreviewSize;  idx++)	{
					//Get STepIDx
					if(m_WalkingTime + (idx+1)*TIME_UNIT > m_StepData[step_data_size -1].TimeData.dAbsStepTime)
						m_StepIdxData(idx) = NO_STEP_IDX;
					else {
						for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++) {
							if(m_WalkingTime + (idx+1)*TIME_UNIT <= m_StepData[step_idx].TimeData.dAbsStepTime)
								break;
						}//for end
						m_StepIdxData(idx) = step_idx;
						previous_step_idx = step_idx;
					}// if else end
					///////////
				}
			}

		}
		else {
			for(int idx = 0; idx < m_PreviewSize;  idx++)	{
				//Get STepIDx
				if(m_WalkingTime + (idx+1)*TIME_UNIT > m_StepData[step_data_size -1].TimeData.dAbsStepTime)
					m_StepIdxData(idx) = NO_STEP_IDX;
				else {
					for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++) {
						if(m_WalkingTime + (idx+1)*TIME_UNIT <= m_StepData[step_idx].TimeData.dAbsStepTime)
							break;
					}//for end
					m_StepIdxData(idx) = step_idx;
					previous_step_idx = step_idx;
				}// if else end
				///////////
			}
		}

	}


	if(m_StepIdxData(m_PreviewSize - 1) != NO_STEP_IDX) {
		if(GetNumofRemainingUnreservedStepData() != 0) {
			m_CurrentStepDataStatus = StepDataStatus1;
			m_ReferenceStepDataforAddition = m_StepData[m_StepIdxData(m_PreviewSize-1)];
			//m_ReferenceStepDataforAddition.TimeData.bWalkingState = InWalking;
		}
		else {
			m_CurrentStepDataStatus = StepDataStatus2;
			m_ReferenceStepDataforAddition = m_StepData[m_StepIdxData(m_PreviewSize-1)];
			//m_ReferenceStepDataforAddition.TimeData.bWalkingState = InWalking;
		}
	}
	else {
		if(m_StepIdxData(0) != NO_STEP_IDX) {
			m_ReferenceStepDataforAddition = m_StepData[m_StepIdxData(0)];
			m_ReferenceStepDataforAddition.TimeData.bWalkingState = InWalkingEnding;
			m_ReferenceStepDataforAddition.TimeData.dAbsStepTime += m_PreviewTime;

			m_CurrentStepDataStatus = StepDataStatus3;
		}
		else {
			m_ReferenceStepDataforAddition.TimeData.bWalkingState = InWalkingEnding;
			m_ReferenceStepDataforAddition.TimeData.dAbsStepTime = m_WalkingTime + m_PreviewTime;
			m_CurrentStepDataStatus = StepDataStatus4;
		}
	}
}

void PreviewControlWalking::CalcRefZMP()
{
	int ref_zmp_idx = 0;
	int step_idx = 0;
	if(m_WalkingTime == 0) {
		if((m_StepIdxData(ref_zmp_idx) == NO_STEP_IDX)/* && (m_StepData.size() == 0)*/)
		{
			m_ZMP_Reference_X.fill((m_PresentLeftFootPosition.x + m_PresentRightFootPosition.x)*0.5);
			m_ZMP_Reference_Y.fill((m_PresentLeftFootPosition.y + m_PresentRightFootPosition.y)*0.5);
			return;
		}

		for(ref_zmp_idx = 0; ref_zmp_idx < m_PreviewSize;  ref_zmp_idx++)
		{
			step_idx = m_StepIdxData(ref_zmp_idx);
			if(step_idx == NO_STEP_IDX)
			{
				m_ZMP_Reference_X(ref_zmp_idx, 0) = m_ZMP_Reference_X(ref_zmp_idx - 1, 0);
				m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_ZMP_Reference_Y(ref_zmp_idx - 1, 0);
			}
			else
			{
				if(m_StepData[step_idx].TimeData.bWalkingState == InWalking)
				{
					if( m_StepData[step_idx].PositionData.bMovingFoot == RFootMove ) {
						m_ZMP_Reference_X(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
						m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
					}
					else if( m_StepData[step_idx].PositionData.bMovingFoot == LFootMove ) {
						m_ZMP_Reference_X(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
						m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
					}
					else if( m_StepData[step_idx].PositionData.bMovingFoot == NFootMove ) {
						m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
						m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
					}
					else {
						m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
						m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
					}
				}
				else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
				}
				else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
				}
				else {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
				}
			}
		}
		m_CurrentStartIdxforZMPRef = 0;
	}
	else {
		step_idx = m_StepIdxData(m_PreviewSize - 1);

		if(m_CurrentStartIdxforZMPRef == 0)
			ref_zmp_idx = m_PreviewSize - 1;
		else
			ref_zmp_idx = m_CurrentStartIdxforZMPRef - 1;

		if(step_idx == NO_STEP_IDX)
		{
			//			if(ref_zmp_idx == 0) {
			//				m_ZMP_Reference_X(ref_zmp_idx, 0) = m_ZMP_Reference_X(m_PreviewSize - 1, 0);
			//				m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_ZMP_Reference_Y(m_PreviewSize - 1, 0);
			//			}
			//			else {
			//				m_ZMP_Reference_X(ref_zmp_idx, 0) = m_ZMP_Reference_X(ref_zmp_idx - 1, 0);
			//				m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_ZMP_Reference_Y(ref_zmp_idx - 1, 0);
			//			}
			m_ZMP_Reference_X(ref_zmp_idx, 0) = 0.5*(m_ReferenceStepDataforAddition.PositionData.stRightFootPosition.x + m_ReferenceStepDataforAddition.PositionData.stLeftFootPosition.x);
			m_ZMP_Reference_Y(ref_zmp_idx, 0) = 0.5*(m_ReferenceStepDataforAddition.PositionData.stRightFootPosition.y + m_ReferenceStepDataforAddition.PositionData.stLeftFootPosition.y);
		}
		else
		{
			if(m_StepData[step_idx].TimeData.bWalkingState == InWalking)
			{
				if( m_StepData[step_idx].PositionData.bMovingFoot == RFootMove ) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_X_ZMP_CenterShift;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_Y_ZMP_CenterShift - m_Y_ZMP_Convergence;
				}
				else if( m_StepData[step_idx].PositionData.bMovingFoot == LFootMove ) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.x + m_X_ZMP_CenterShift;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = m_StepData[step_idx].PositionData.stRightFootPosition.y + m_Y_ZMP_CenterShift + m_Y_ZMP_Convergence;
				}
				else if( m_StepData[step_idx].PositionData.bMovingFoot == NFootMove ) {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
				}
				else {
					m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
					m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
				}
			}
			else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingStarting) {
				m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
				m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
			}
			else if(m_StepData[step_idx].TimeData.bWalkingState == InWalkingEnding) {
				m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
				m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
			}
			else {
				m_ZMP_Reference_X(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.x + m_StepData[step_idx].PositionData.stRightFootPosition.x)*0.5;
				m_ZMP_Reference_Y(ref_zmp_idx, 0) = (m_StepData[step_idx].PositionData.stLeftFootPosition.y + m_StepData[step_idx].PositionData.stRightFootPosition.y)*0.5;
			}
		}
	}
}

void PreviewControlWalking::SetRefZMPDecisionParameter(double X_ZMP_CenterShift, double Y_ZMP_CenterShift, double Y_ZMP_Convergence)
{
	m_X_ZMP_CenterShift = X_ZMP_CenterShift;
	m_Y_ZMP_CenterShift = Y_ZMP_CenterShift;
	m_Y_ZMP_Convergence = Y_ZMP_Convergence;
}


void PreviewControlWalking::SetInitForceTorque(double init_right_fx_N,  double init_right_fy_N,  double init_right_fz_N,
		double init_right_Tx_Nm,double init_right_Ty_Nm,double  init_right_Tz_Nm,
		double init_left_fx_N, double init_left_fy_N, double init_left_fz_N,
		double init_left_Tx_Nm, double init_left_Ty_Nm, double init_left_Tz_Nm)
{
	m_init_right_fx_N = init_right_fx_N;   m_init_right_fy_N = init_right_fy_N;   m_init_right_fz_N = init_right_fz_N;
	m_init_right_Tx_Nm = init_right_Tx_Nm; m_init_right_Ty_Nm = init_right_Ty_Nm; m_init_right_Tz_Nm = init_right_Tz_Nm;

	m_init_left_fx_N = init_left_fx_N;   m_init_left_fy_N = init_left_fy_N;   m_init_left_fz_N = init_left_fz_N;
	m_init_left_Tx_Nm = init_left_Tx_Nm; m_init_left_Ty_Nm = init_left_Ty_Nm; m_init_left_Tz_Nm = init_left_Tz_Nm;
}

void PreviewControlWalking::SetInitForceOntheGround(double right_fx_on_gnd_N, double  right_fy_on_gnd_N,double  right_fz_on_gnd_N,
		double right_tx_on_gnd_N ,double right_ty_on_gnd_N , double right_tz_on_gnd_N,
		double left_fx_on_gnd_N ,  double left_fy_on_gnd_N ,  double left_fz_on_gnd_N,
		double left_tx_on_gnd_N , double left_ty_on_gnd_N ,double left_tz_on_gnd_N)
{
	//	//each calib
	//	m_right_ft_scale_factor = (fabs(m_right_dsp_fz_N) - 0.5*9.8)/(right_fz_on_gnd_N - m_init_right_fz_N);
	//	m_left_ft_scale_factor  = (fabs(m_left_dsp_fz_N)  - 0.5*9.8)/(left_fz_on_gnd_N - m_init_left_fz_N);

	//both calib
	m_right_ft_scale_factor = (fabs(m_right_dsp_fz_N) + fabs(m_left_dsp_fz_N) - 0.5*9.8 - 0.5*9.8)/(right_fz_on_gnd_N + left_fz_on_gnd_N - m_init_right_fz_N - m_init_left_fz_N);
	m_left_ft_scale_factor  = (fabs(m_right_dsp_fz_N) + fabs(m_left_dsp_fz_N) - 0.5*9.8 - 0.5*9.8)/(right_fz_on_gnd_N + left_fz_on_gnd_N - m_init_right_fz_N - m_init_left_fz_N);

	printf("r_scale : %f   l_scale : %f\n", m_right_ft_scale_factor, m_left_ft_scale_factor);

	//	m_right_dsp_fz_N = 475.0*0.5;
	//	m_right_ssp_fz_N = 475.0;
	//	m_left_dsp_fz_N = 475.0*0.5;
	//	m_left_ssp_fz_N = 475.0;
	//	m_right_dsp_fz_N = -1.0*550.0*0.5;
	//	m_right_ssp_fz_N = -1.0*550.0;
	//	m_left_dsp_fz_N = -1.0*550.0*0.5;
	//	m_left_ssp_fz_N = -1.0*550.0;

	//	double f_on_gnd_N = (right_fx_on_gnd_N + left_fx_on_gnd_N - m_init_right_fx_N - m_init_left_fx_N)*(right_fx_on_gnd_N + left_fx_on_gnd_N - m_init_right_fx_N - m_init_left_fx_N)
	//			+(right_fy_on_gnd_N + left_fy_on_gnd_N - m_init_right_fy_N - m_init_left_fy_N)*(right_fy_on_gnd_N + left_fy_on_gnd_N - m_init_right_fy_N - m_init_left_fy_N)
	//			+(right_fz_on_gnd_N + left_fz_on_gnd_N - m_init_right_fz_N - m_init_left_fz_N)*(right_fz_on_gnd_N + left_fz_on_gnd_N - m_init_right_fz_N - m_init_left_fz_N);
	//
	//	f_on_gnd_N = sqrt(f_on_gnd_N);
	//
	//	m_left_ft_scale_factor = m_right_ft_scale_factor = (m_right_dsp_fz_N + m_left_dsp_fz_N)/(f_on_gnd_N);
	//	m_left_ft_scale_factor = 1.0;
	//	m_right_ft_scale_factor = 1.0;
	//m_left_ft_scale_factor = m_right_ft_scale_factor = (m_right_dsp_fz_N + m_left_dsp_fz_N)/(right_fz_on_gnd_N + left_fz_on_gnd_N - m_init_right_fz_N - m_init_left_fz_N);
}


double PreviewControlWalking::GetDampingControllerOutput(double desired, double present, double previous_output, double goal_settling_time)
{
	double cut_off_freq = 1.0/goal_settling_time;
	double alpha = 1.0;
	if(TIME_UNIT < 1.0)
		alpha = (2.0*M_PI*cut_off_freq*TIME_UNIT)/(1.0+2.0*M_PI*cut_off_freq*TIME_UNIT);
	else
		alpha = (2.0*M_PI*cut_off_freq*TIME_UNIT/1000.0)/(1.0+2.0*M_PI*cut_off_freq*TIME_UNIT/1000.0);

	double output =  alpha*(desired - present) + (1.0 - alpha)*previous_output;

	return output;
}

//double PreviewControlWalking::GetHipRightRollDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
//{
//	static double previous_hip_right_roll_modification = 0.0;
//
//	previous_hip_right_roll_modification = GetDampingControllerOutput(desired, present, previous_hip_right_roll_modification, goal_settling_time);
//
//	return gain*previous_hip_right_roll_modification;
//}

//double PreviewControlWalking::GetHipLeftRollDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
//{
//	static double previous_hip_left_roll_modification = 0.0;
//
//	previous_hip_left_roll_modification = GetDampingControllerOutput(desired, present, previous_hip_left_roll_modification, goal_settling_time);
//
//	return gain*previous_hip_left_roll_modification;
//}

double PreviewControlWalking::GetHipRollDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_hip_roll_modification = 0.0;

	previous_hip_roll_modification = GetDampingControllerOutput(desired, present, previous_hip_roll_modification, goal_settling_time);

	return gain*previous_hip_roll_modification;
}


double PreviewControlWalking::GetHipPitchDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_hip_pitch_modification = 0.0;

	previous_hip_pitch_modification = GetDampingControllerOutput(desired, present, previous_hip_pitch_modification, goal_settling_time);

	return gain*previous_hip_pitch_modification;
}

double PreviewControlWalking::GetAnkleRollDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_ankle_roll_modification = 0.0;

	previous_ankle_roll_modification = GetDampingControllerOutput(desired, present, previous_ankle_roll_modification, goal_settling_time);

	return gain*previous_ankle_roll_modification;
}

double PreviewControlWalking::GetAnklePitchDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_ankle_pitch_modification = 0.0;

	previous_ankle_pitch_modification = GetDampingControllerOutput(desired, present, previous_ankle_pitch_modification, goal_settling_time);

	return gain*previous_ankle_pitch_modification;
}

double PreviewControlWalking::GetZDampingControllOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_z_modification = 0.0;
	previous_z_modification = GetDampingControllerOutput(desired, present, previous_z_modification, goal_settling_time);
	return gain*previous_z_modification;
}


double PreviewControlWalking::GetRightXDampingControlOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_r_x_modification = 0.0;
	previous_r_x_modification = GetDampingControllerOutput(desired, present, previous_r_x_modification, goal_settling_time);
	return gain*previous_r_x_modification;
}

double PreviewControlWalking::GetRightYDampingControlOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_r_y_modification = 0.0;
	previous_r_y_modification = GetDampingControllerOutput(desired, present, previous_r_y_modification, goal_settling_time);
	return gain*previous_r_y_modification;
}

double PreviewControlWalking::GetLeftXDampingControlOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_l_x_modification = 0.0;
	previous_l_x_modification = GetDampingControllerOutput(desired, present, previous_l_x_modification, goal_settling_time);
	return gain*previous_l_x_modification;
}

double PreviewControlWalking::GetLeftYDampingControlOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_l_y_modification = 0.0;
	previous_l_y_modification = GetDampingControllerOutput(desired, present, previous_l_y_modification, goal_settling_time);
	return gain*previous_l_y_modification;
}


double PreviewControlWalking::GetRightAnkleRollMomentDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_right_ankle_roll_moment_modification = 0.0;
	previous_right_ankle_roll_moment_modification = GetDampingControllerOutput(desired, present, previous_right_ankle_roll_moment_modification, goal_settling_time);
	return gain*previous_right_ankle_roll_moment_modification;
}

double PreviewControlWalking::GetRightAnklePitchMomentDampingControllerOutput(double desired, double present, double goal_settling_time, double gain)
{
	static double previous_right_ankle_pitch_moment_modification = 0.0;
	previous_right_ankle_pitch_moment_modification = GetDampingControllerOutput(desired, present, previous_right_ankle_pitch_moment_modification, goal_settling_time);
	return gain*previous_right_ankle_pitch_moment_modification;
}

double PreviewControlWalking::GetLeftAnkleRollMomentDampingControllerOutput(double desired, double present,  double goal_settling_time, double gain)
{
	static double previous_left_ankle_roll_moment_modification = 0.0;
	previous_left_ankle_roll_moment_modification = GetDampingControllerOutput(desired, present, previous_left_ankle_roll_moment_modification, goal_settling_time);
	return gain*previous_left_ankle_roll_moment_modification;
}

double PreviewControlWalking::GetLeftAnklePitchMomentDampingControllerOutput(double desired, double present,  double goal_settling_time, double gain)
{
	static double previous_left_ankle_pitch_moment_modification = 0.0;
	previous_left_ankle_pitch_moment_modification = GetDampingControllerOutput(desired, present, previous_left_ankle_pitch_moment_modification, goal_settling_time);
	return gain*previous_left_ankle_pitch_moment_modification;
}
//
//
//Pose3D PreviewControlWalking::GetRightLegAxisDampingControllerOutput(Pose3D epr_desired, Pose3D epr_present, double goal_time_constant, double gain)
//{
//	static double prev_epr_x_modification = 0.0;
//	static double prev_epr_y_modification = 0.0;
//	static double prev_epr_z_modification = 0.0;
//	static double prev_epr_roll_modification = 0.0;
//	static double prev_epr_pitch_modification = 0.0;
//	static double prev_epr_yaw_modification = 0.0;
//	static Pose3D epr_modification;
//
//	prev_epr_x_modification     = GetDampingControllerOutput(epr_desired.x,     epr_present.x,     prev_epr_x_modification,     goal_time_constant);
//	prev_epr_y_modification     = GetDampingControllerOutput(epr_desired.y,     epr_present.y,     prev_epr_y_modification,     goal_time_constant);
//	prev_epr_z_modification     = GetDampingControllerOutput(epr_desired.z,     epr_present.z,     prev_epr_z_modification,     goal_time_constant);
//	prev_epr_roll_modification  = GetDampingControllerOutput(epr_desired.roll,  epr_present.roll,  prev_epr_roll_modification,  goal_time_constant);
//	prev_epr_pitch_modification = GetDampingControllerOutput(epr_desired.pitch, epr_present.pitch, prev_epr_pitch_modification, goal_time_constant);
//	prev_epr_yaw_modification   = GetDampingControllerOutput(epr_desired.yaw,   epr_present.yaw,   prev_epr_yaw_modification,   goal_time_constant);
//
//	epr_modification.x     = gain*prev_epr_x_modification;
//	epr_modification.y     = gain*prev_epr_y_modification;
//	epr_modification.z     = gain*prev_epr_z_modification;
//	epr_modification.roll  = gain*prev_epr_roll_modification;
//	epr_modification.pitch = gain*prev_epr_pitch_modification;
//	epr_modification.yaw   = gain*prev_epr_yaw_modification;
//
//	return epr_modification;
//}
//
//Pose3D PreviewControlWalking::GetLeftLegAxisDampingControllerOutput(Pose3D epl_desired, Pose3D epl_present, double goal_time_constant, double gain)
//{
//	static double prev_epl_x_modification = 0.0;
//	static double prev_epl_y_modification = 0.0;
//	static double prev_epl_z_modification = 0.0;
//	static double prev_epl_roll_modification = 0.0;
//	static double prev_epl_pitch_modification = 0.0;
//	static double prev_epl_yaw_modification = 0.0;
//	static Pose3D epl_modification;
//
//	prev_epl_x_modification     = GetDampingControllerOutput(epl_desired.x,     epl_present.x,     prev_epl_x_modification,     goal_time_constant);
//	prev_epl_y_modification     = GetDampingControllerOutput(epl_desired.y,     epl_present.y,     prev_epl_y_modification,     goal_time_constant);
//	prev_epl_z_modification     = GetDampingControllerOutput(epl_desired.z,     epl_present.z,     prev_epl_z_modification,     goal_time_constant);
//	prev_epl_roll_modification  = GetDampingControllerOutput(epl_desired.roll,  epl_present.roll,  prev_epl_roll_modification,  goal_time_constant);
//	prev_epl_pitch_modification = GetDampingControllerOutput(epl_desired.pitch, epl_present.pitch, prev_epl_pitch_modification, goal_time_constant);
//	prev_epl_yaw_modification   = GetDampingControllerOutput(epl_desired.yaw,   epl_present.yaw,   prev_epl_yaw_modification,   goal_time_constant);
//
//	epl_modification.x     = gain*prev_epl_x_modification;
//	epl_modification.y     = gain*prev_epl_y_modification;
//	epl_modification.z     = gain*prev_epl_z_modification;
//	epl_modification.roll  = gain*prev_epl_roll_modification;
//	epl_modification.pitch = gain*prev_epl_pitch_modification;
//	epl_modification.yaw   = gain*prev_epl_yaw_modification;
//
//	return epl_modification;
//}
//
//
//double PreviewControlWalking::GetXLandingDampingControllerOutput(double desired, double present,  double goal_settling_time, double gain)
//{
//	static double previous_cob_x_modification = 0.0;
//	previous_cob_x_modification = GetDampingControllerOutput(desired, present, previous_cob_x_modification, goal_settling_time);
//	return gain*previous_cob_x_modification;
//}
//
//double PreviewControlWalking::GetYLandingDampingControllerOutput(double desired, double present,  double goal_settling_time, double gain)
//{
//	static double previous_cob_y_modification = 0.0;
//	previous_cob_y_modification = GetDampingControllerOutput(desired, present, previous_cob_y_modification, goal_settling_time);
//	return gain*previous_cob_y_modification;
//}
//
//
void PreviewControlWalking::Process()
{
	static Pose3D epr, epl;
	static Pose3D epr_for_balance, epl_for_balance;

	if(!m_Ctrl_Running) {
		return;
	}
	else {
		pthread_mutex_lock(&m_mutex_lock);

		CalcStepIdxData();
		CalcRefZMP();
		//		//Original LIPM
		//		u_x = -K*x_LIPM + x_feed_forward_term;
		//		x_LIPM = A*x_LIPM + b*u_x;
		//
		//		u_y = -K*y_LIPM + y_feed_forward_term;
		//		y_LIPM = A*y_LIPM + b*u_y;

		//Calc LIPM with Integral
		matd x_feed_forward_term2; //f_Preview*m_ZMP_Reference_X;
		matd y_feed_forward_term2; //f_Preview*m_ZMP_Reference_Y;

		x_feed_forward_term2.resize(1,1);
		x_feed_forward_term2.fill(0.0);
		y_feed_forward_term2.resize(1,1);
		y_feed_forward_term2.fill(0.0);

		for(int i = 0; i < m_PreviewSize; i++)
		{
			if(m_CurrentStartIdxforZMPRef + i < m_PreviewSize) {
				x_feed_forward_term2(0,0) += f_(i)*m_ZMP_Reference_X(m_CurrentStartIdxforZMPRef + i, 0);
				y_feed_forward_term2(0,0) += f_(i)*m_ZMP_Reference_Y(m_CurrentStartIdxforZMPRef + i, 0);
			}
			else {
				x_feed_forward_term2(0,0) += f_(i)*m_ZMP_Reference_X(m_CurrentStartIdxforZMPRef + i - m_PreviewSize, 0);
				y_feed_forward_term2(0,0) += f_(i)*m_ZMP_Reference_Y(m_CurrentStartIdxforZMPRef + i - m_PreviewSize, 0);
			}
		}

		sum_of_cx += c(0,0)*x_LIPM(0,0) +  c(0,1)*x_LIPM(1,0) +  c(0,2)*x_LIPM(2,0);
		sum_of_cy += c(0,0)*y_LIPM(0,0) +  c(0,1)*y_LIPM(1,0) +  c(0,2)*y_LIPM(2,0);

		u_x(0,0) = -K_s_*(sum_of_cx - sum_of_zmp_x) - (K_x_(0,0)*x_LIPM(0,0) + K_x_(0,1)*x_LIPM(1,0) + K_x_(0,2)*x_LIPM(2,0)) + x_feed_forward_term2(0,0);
		u_y(0,0) = -K_s_*(sum_of_cy - sum_of_zmp_y) - (K_x_(0,0)*y_LIPM(0,0) + K_x_(0,1)*y_LIPM(1,0) + K_x_(0,2)*y_LIPM(2,0)) + y_feed_forward_term2(0,0);
		x_LIPM = A*x_LIPM + b*u_x;
		y_LIPM = A*y_LIPM + b*u_y;


		zmp_ref_x_at_this_time = m_ZMP_Reference_X(m_CurrentStartIdxforZMPRef, 0);
		zmp_ref_y_at_this_time = m_ZMP_Reference_Y(m_CurrentStartIdxforZMPRef, 0);

		sum_of_zmp_x += m_ZMP_Reference_X(m_CurrentStartIdxforZMPRef, 0);
		sum_of_zmp_y += m_ZMP_Reference_Y(m_CurrentStartIdxforZMPRef, 0);

		m_PresentBodyPosition.x = x_LIPM(0,0);
		m_PresentBodyPosition.y = y_LIPM(0,0);

		m_ReferenceStepDataforAddition.PositionData.stBodyPosition.x = x_LIPM(0,0);
		m_ReferenceStepDataforAddition.PositionData.stBodyPosition.y = y_LIPM(0,0);

		m_CurrentStartIdxforZMPRef++;
		if(m_CurrentStartIdxforZMPRef == (m_PreviewSize))
			m_CurrentStartIdxforZMPRef = 0;

		double body_roll_swap = 0;

		int detail_balance_time_idx = 0;
		if((m_StepData.size() != 0) && m_Real_Running) {
			double body_move_periodTime = m_StepData[0].TimeData.dAbsStepTime - m_ReferenceTime;
			double wp_move_amp = m_StepData[0].PositionData.dWaistYawAngle - m_PreviousStepWaistYawAngleRad;
			double wp_move_amp_shift = m_PreviousStepWaistYawAngleRad;

			double bz_move_amp = m_StepData[0].PositionData.stBodyPosition.z - m_PreviousStepBodyPosition.z;
			double bz_move_amp_shift = m_PreviousStepBodyPosition.z;

			double ba_move_amp = m_StepData[0].PositionData.stBodyPosition.roll - m_PreviousStepBodyPosition.roll;
			double ba_move_amp_shift = m_PreviousStepBodyPosition.roll;

			double bb_move_amp = m_StepData[0].PositionData.stBodyPosition.pitch - m_PreviousStepBodyPosition.pitch;
			double bb_move_amp_shift = m_PreviousStepBodyPosition.pitch;

			double bc_move_amp = m_StepData[0].PositionData.stBodyPosition.yaw - m_PreviousStepBodyPosition.yaw;
			double bc_move_amp_shift = m_PreviousStepBodyPosition.yaw;

			double z_swap_amp = 0.5*(m_StepData[0].PositionData.dZ_Swap_Amplitude);
			double z_swap_amp_shift = z_swap_amp;
			double z_swap_phase_shift = M_PI*0.5;

			double body_roll_swap_dir = 1.0;
			double body_roll_swap_amp = 0.5*(HIP_ROLL_FEEDFORWARD_ANGLE_RAD);
			double body_roll_swap_amp_shift = body_roll_swap_amp;

			if(bc_move_amp >= M_PI)
				bc_move_amp -= 2.0*M_PI;
			else if(bc_move_amp <= -M_PI)
				bc_move_amp += 2.0*M_PI;

			detail_balance_time_idx = (int)( (body_move_periodTime - m_WalkingTime + m_ReferenceTime) / (double)TIME_UNIT );
			if(detail_balance_time_idx >= m_PreviewSize)
				detail_balance_time_idx = m_PreviewSize - 1;

			double z_swap  = wsin(m_WalkingTime - m_ReferenceTime, body_move_periodTime, z_swap_phase_shift, z_swap_amp, z_swap_amp_shift);

			double wp_move = wsigmoid(m_WalkingTime - m_ReferenceTime, body_move_periodTime, 0, wp_move_amp, wp_move_amp_shift, 1.0, 1.0);
			double bz_move = wsigmoid(m_WalkingTime - m_ReferenceTime, body_move_periodTime, 0, bz_move_amp, bz_move_amp_shift, 1.0, 1.0);
			double ba_move = wsigmoid(m_WalkingTime - m_ReferenceTime, body_move_periodTime, 0, ba_move_amp, ba_move_amp_shift, 1.0, 1.0);
			double bb_move = wsigmoid(m_WalkingTime - m_ReferenceTime, body_move_periodTime, 0, bb_move_amp, bb_move_amp_shift, 1.0, 1.0);
			double bc_move = wsigmoid(m_WalkingTime - m_ReferenceTime, body_move_periodTime, 0, bc_move_amp, bc_move_amp_shift, 1.0, 1.0);

			m_PresentWaistYawAngleRad = wp_move;
			m_PresentBodyPosition.z = bz_move + z_swap;
			m_PresentBodyPosition.roll = ba_move;
			m_PresentBodyPosition.pitch = bb_move;
			m_PresentBodyPosition.yaw = bc_move;

			//Feet
			double time = m_WalkingTime - m_ReferenceTime;
			double period_time, dsp_ratio, ssp_ratio, foot_move_period_time, ssp_time_start, ssp_time_end;
			double x_move_amp, y_move_amp, z_move_amp, a_move_amp, b_move_amp, c_move_amp, z_vibe_amp;
			double x_move_amp_shift, y_move_amp_shift, z_move_amp_shift, a_move_amp_shift, b_move_amp_shift, c_move_amp_shift, z_vibe_amp_shift;
			double z_vibe_phase_shift;
			double x_move, y_move, z_move, a_move, b_move, c_move, z_vibe;

			period_time = m_StepData[0].TimeData.dAbsStepTime - m_ReferenceTime;
			dsp_ratio = m_StepData[0].TimeData.dDSPratio;
			ssp_ratio = 1 - dsp_ratio;
			foot_move_period_time = ssp_ratio*period_time;

			ssp_time_start = dsp_ratio*period_time/2.0;
			ssp_time_end = (1 + ssp_ratio)*period_time / 2.0;

			if(m_StepData[0].PositionData.bMovingFoot == RFootMove)
			{
				x_move_amp = (m_StepData[0].PositionData.stRightFootPosition.x - m_PreviousStepRightFootPosition.x);
				x_move_amp_shift = m_PreviousStepRightFootPosition.x;

				y_move_amp = (m_StepData[0].PositionData.stRightFootPosition.y - m_PreviousStepRightFootPosition.y);
				y_move_amp_shift = m_PreviousStepRightFootPosition.y;

				z_move_amp = (m_StepData[0].PositionData.stRightFootPosition.z - m_PreviousStepRightFootPosition.z);
				z_move_amp_shift = m_PreviousStepRightFootPosition.z;

				a_move_amp = (m_StepData[0].PositionData.stRightFootPosition.roll - m_PreviousStepRightFootPosition.roll);
				a_move_amp_shift = m_PreviousStepRightFootPosition.roll;

				b_move_amp = (m_StepData[0].PositionData.stRightFootPosition.pitch - m_PreviousStepRightFootPosition.pitch);
				b_move_amp_shift = m_PreviousStepRightFootPosition.pitch;

				c_move_amp = (m_StepData[0].PositionData.stRightFootPosition.yaw - m_PreviousStepRightFootPosition.yaw);
				c_move_amp_shift = m_PreviousStepRightFootPosition.yaw;

				z_vibe_amp = m_StepData[0].PositionData.dFootHeight*0.5;
				z_vibe_amp_shift = z_vibe_amp;
				z_vibe_phase_shift = M_PI*0.5;

				body_roll_swap_dir = -1.0;
			}
			else if(m_StepData[0].PositionData.bMovingFoot == LFootMove)	{
				x_move_amp = (m_StepData[0].PositionData.stLeftFootPosition.x - m_PreviousStepLeftFootPosition.x);
				x_move_amp_shift = m_PreviousStepLeftFootPosition.x;

				y_move_amp = (m_StepData[0].PositionData.stLeftFootPosition.y - m_PreviousStepLeftFootPosition.y);
				y_move_amp_shift = m_PreviousStepLeftFootPosition.y;

				z_move_amp = (m_StepData[0].PositionData.stLeftFootPosition.z - m_PreviousStepLeftFootPosition.z);
				z_move_amp_shift = m_PreviousStepLeftFootPosition.z;

				a_move_amp = (m_StepData[0].PositionData.stLeftFootPosition.roll - m_PreviousStepLeftFootPosition.roll);
				a_move_amp_shift = m_PreviousStepLeftFootPosition.roll;

				b_move_amp = (m_StepData[0].PositionData.stLeftFootPosition.pitch - m_PreviousStepLeftFootPosition.pitch);
				b_move_amp_shift = m_PreviousStepLeftFootPosition.pitch;

				c_move_amp = (m_StepData[0].PositionData.stLeftFootPosition.yaw - m_PreviousStepLeftFootPosition.yaw);
				c_move_amp_shift = m_PreviousStepLeftFootPosition.yaw;

				z_vibe_amp = m_StepData[0].PositionData.dFootHeight*0.5;
				z_vibe_amp_shift = z_vibe_amp;
				z_vibe_phase_shift = M_PI*0.5;

				body_roll_swap_dir = 1.0;
			}
			else {
				x_move_amp = 0.0;
				x_move_amp_shift = m_PreviousStepLeftFootPosition.x;

				y_move_amp = 0.0;
				y_move_amp_shift = m_PreviousStepLeftFootPosition.y;

				z_move_amp = 0.0;
				z_move_amp_shift = m_PreviousStepLeftFootPosition.z;

				a_move_amp = 0.0;
				a_move_amp_shift = m_PreviousStepLeftFootPosition.roll;

				b_move_amp = 0.0;
				b_move_amp_shift = m_PreviousStepLeftFootPosition.pitch;

				c_move_amp = 0.0;
				c_move_amp_shift = m_PreviousStepLeftFootPosition.yaw;

				z_vibe_amp = 0.0;
				z_vibe_amp_shift = z_vibe_amp;
				z_vibe_phase_shift = M_PI*0.5;

				body_roll_swap_dir = 0.0;
			}

			if(c_move_amp >= M_PI)
				c_move_amp -= 2.0*M_PI;
			else if(c_move_amp <= -M_PI)
				c_move_amp += 2.0*M_PI;


			if( time <= ssp_time_start)
			{
				x_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_x,     m_StepData[0].TimeData.sigmoid_distortion_x);
				y_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_y,     m_StepData[0].TimeData.sigmoid_distortion_y);
				z_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_z,     m_StepData[0].TimeData.sigmoid_distortion_z);
				a_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_roll,  m_StepData[0].TimeData.sigmoid_distortion_roll);
				b_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_pitch, m_StepData[0].TimeData.sigmoid_distortion_pitch);
				c_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_yaw,   m_StepData[0].TimeData.sigmoid_distortion_yaw);

				z_vibe         = wsin(ssp_time_start, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);
				body_roll_swap = wsin(ssp_time_start, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, body_roll_swap_amp, body_roll_swap_amp_shift);
				if(m_StepData[0].PositionData.bMovingFoot == RFootMove)
				{
					m_Balancing_Index = 1;
				}
				else if(m_StepData[0].PositionData.bMovingFoot == LFootMove)	{
					m_Balancing_Index = 5;
				}
				else {
					m_Balancing_Index = 0;
				}
			}
			else if(time <= ssp_time_end) {
				x_move = wsigmoid(time, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_x,     m_StepData[0].TimeData.sigmoid_distortion_x);
				y_move = wsigmoid(time, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_y,     m_StepData[0].TimeData.sigmoid_distortion_y);
				z_move = wsigmoid(time, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_z,     m_StepData[0].TimeData.sigmoid_distortion_z);
				a_move = wsigmoid(time, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_roll,  m_StepData[0].TimeData.sigmoid_distortion_roll);
				b_move = wsigmoid(time, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_pitch, m_StepData[0].TimeData.sigmoid_distortion_pitch);
				c_move = wsigmoid(time, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_yaw,   m_StepData[0].TimeData.sigmoid_distortion_yaw);

				z_vibe         = wsin(time, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, z_vibe_amp,         z_vibe_amp_shift);
				body_roll_swap = wsin(time, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, body_roll_swap_amp, body_roll_swap_amp_shift);

				if(m_StepData[0].PositionData.bMovingFoot == RFootMove)
				{
					if(time <= (ssp_time_end + ssp_time_start)*0.5)
						m_Balancing_Index = 2;
					else
						m_Balancing_Index = 3;
				}
				else if(m_StepData[0].PositionData.bMovingFoot == LFootMove)	{
					if(time <= (ssp_time_end + ssp_time_start)*0.5)
						m_Balancing_Index = 6;
					else
						m_Balancing_Index = 7;
				}
				else {
					m_Balancing_Index = 0;
				}
			}
			else {
				x_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_x,     m_StepData[0].TimeData.sigmoid_distortion_x);
				y_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_y,     m_StepData[0].TimeData.sigmoid_distortion_y);
				z_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_z,     m_StepData[0].TimeData.sigmoid_distortion_z);
				a_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_roll,  m_StepData[0].TimeData.sigmoid_distortion_roll);
				b_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_pitch, m_StepData[0].TimeData.sigmoid_distortion_pitch);
				c_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, m_StepData[0].TimeData.sigmoid_ratio_yaw,   m_StepData[0].TimeData.sigmoid_distortion_yaw);

				z_vibe         = wsin(ssp_time_end, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, z_vibe_amp,         z_vibe_amp_shift);
				body_roll_swap = wsin(ssp_time_end, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, body_roll_swap_amp, body_roll_swap_amp_shift);
				if(m_StepData[0].PositionData.bMovingFoot == RFootMove)
				{
					m_Balancing_Index = 4;
				}
				else if(m_StepData[0].PositionData.bMovingFoot == LFootMove)	{
					m_Balancing_Index = 8;
				}
				else {
					m_Balancing_Index = 0;
				}
			}

			body_roll_swap = body_roll_swap_dir * body_roll_swap;

//			printf("m_Stepdata.size :  %d, time : %f, Foot : %d  x_move: %f %f\n",
//					m_StepData.size(), m_StepData[0].TimeData.dAbsStepTime, m_StepData[0].PositionData.bMovingFoot, x_move, time);
			if(m_StepData[0].PositionData.bMovingFoot == RFootMove)
			{
				m_PresentRightFootPosition.x = x_move;
				m_PresentRightFootPosition.y = y_move;
				m_PresentRightFootPosition.z = z_move + z_vibe;
				m_PresentRightFootPosition.roll = a_move;
				m_PresentRightFootPosition.pitch = b_move;
				m_PresentRightFootPosition.yaw = c_move;

				m_PresentLeftFootPosition = m_StepData[0].PositionData.stLeftFootPosition;
			}
			else if(m_StepData[0].PositionData.bMovingFoot == LFootMove)	{
				m_PresentRightFootPosition = m_StepData[0].PositionData.stRightFootPosition;

				m_PresentLeftFootPosition.x = x_move;
				m_PresentLeftFootPosition.y = y_move;
				m_PresentLeftFootPosition.z = z_move + z_vibe;
				m_PresentLeftFootPosition.roll = a_move;
				m_PresentLeftFootPosition.pitch = b_move;

				m_PresentLeftFootPosition.yaw = c_move;
			}
			else {
				//printf("%d %f\n", m_WalkingTime, m_PresentLeftFootPosition.x );
				m_PresentRightFootPosition = m_StepData[0].PositionData.stRightFootPosition;
				m_PresentLeftFootPosition = m_StepData[0].PositionData.stLeftFootPosition;
			}

			m_ShoulerSwingGain = m_StepData[0].PositionData.dShoulderSwingGain;
			m_ElbowSwingGain = m_StepData[0].PositionData.dElbowSwingGain;

			m_WalkingTime += TIME_UNIT;

			//fprintf(stderr, "%d, %f\n", m_WalkingTime, m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime);
			if(m_WalkingTime > m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime - 0.5) {
//				m_WalkingTime = m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime;
				m_Real_Running = false;
				CalcStepIdxData();
				pthread_mutex_unlock(&m_mutex_lock);
				LocalizeAllWalkingParameter();
//				printf("Localize\n");
			}

			if(m_Balancing_Index == 0 || m_Balancing_Index == 9)
			{
				m_Left_Fz_Sigmoid_StartTime = m_WalkingTime;
				m_Left_Fz_Sigmoid_EndTime   = m_WalkingTime;
				m_Left_Fz_Sigmoid_Target  = m_left_dsp_fz_N;
				m_Left_Fz_Sigmoid_Shift   = m_left_dsp_fz_N;
			}
			else if(m_Balancing_Index == 1 )
			{
				m_Left_Fz_Sigmoid_EndTime = ssp_time_start + m_ReferenceTime;
				m_Left_Fz_Sigmoid_Target = m_left_ssp_fz_N;
			}
			else if(m_Balancing_Index == 4 )
			{
				m_Left_Fz_Sigmoid_StartTime = ssp_time_end + m_ReferenceTime;
				m_Left_Fz_Sigmoid_Shift = m_left_ssp_fz_N;
				if(m_StepData.size() > 1)
				{
					if(m_StepData[1].PositionData.bMovingFoot == NFootMove) {
						m_Left_Fz_Sigmoid_Target = m_left_dsp_fz_N;
						m_Left_Fz_Sigmoid_EndTime = m_StepData[0].TimeData.dAbsStepTime;
					}
					else if(m_StepData[1].PositionData.bMovingFoot == LFootMove) {
						m_Left_Fz_Sigmoid_Target = 0.0;
						m_Left_Fz_Sigmoid_EndTime = (m_StepData[1].TimeData.dAbsStepTime - m_StepData[0].TimeData.dAbsStepTime)*0.5*m_StepData[1].TimeData.dDSPratio + m_StepData[0].TimeData.dAbsStepTime;
					}
					else {
						m_Left_Fz_Sigmoid_Target = m_left_ssp_fz_N;
						m_Left_Fz_Sigmoid_EndTime = (m_StepData[1].TimeData.dAbsStepTime - m_StepData[0].TimeData.dAbsStepTime)*0.5*m_StepData[1].TimeData.dDSPratio + m_StepData[0].TimeData.dAbsStepTime;
					}
				}
				else {
					m_Left_Fz_Sigmoid_Target = m_left_dsp_fz_N;
					m_Left_Fz_Sigmoid_EndTime = m_StepData[0].TimeData.dAbsStepTime;
				}

			}
			else if(m_Balancing_Index == 5 )
			{
				m_Left_Fz_Sigmoid_EndTime = ssp_time_start + m_ReferenceTime;
				m_Left_Fz_Sigmoid_Target = 0.0;
			}
			else if(m_Balancing_Index == 8)
			{
				m_Left_Fz_Sigmoid_StartTime = ssp_time_end + m_ReferenceTime;
				m_Left_Fz_Sigmoid_Shift = 0.0;
				if(m_StepData.size() > 1)
				{
					if(m_StepData[1].PositionData.bMovingFoot == NFootMove) {
						m_Left_Fz_Sigmoid_Target = m_left_dsp_fz_N;
						m_Left_Fz_Sigmoid_EndTime = m_StepData[0].TimeData.dAbsStepTime;
					}
					else if(m_StepData[1].PositionData.bMovingFoot == LFootMove) {
						m_Left_Fz_Sigmoid_Target = 0.0;
						m_Left_Fz_Sigmoid_EndTime = (m_StepData[1].TimeData.dAbsStepTime - m_StepData[0].TimeData.dAbsStepTime)*0.5*m_StepData[1].TimeData.dDSPratio + m_StepData[0].TimeData.dAbsStepTime;
					}
					else {
						m_Left_Fz_Sigmoid_Target = m_left_ssp_fz_N;
						m_Left_Fz_Sigmoid_EndTime = (m_StepData[1].TimeData.dAbsStepTime - m_StepData[0].TimeData.dAbsStepTime)*0.5*m_StepData[1].TimeData.dDSPratio + m_StepData[0].TimeData.dAbsStepTime;
					}
				}
				else {
					m_Left_Fz_Sigmoid_Target = m_left_dsp_fz_N;
					m_Left_Fz_Sigmoid_EndTime = m_StepData[0].TimeData.dAbsStepTime;
				}
			}
			else {
				m_Left_Fz_Sigmoid_StartTime = m_WalkingTime;
				m_Left_Fz_Sigmoid_EndTime   = m_WalkingTime;
			}

		}



		pthread_mutex_unlock(&m_mutex_lock);

		matGtoCOB = GetTransformMatrix(m_PresentBodyPosition.x, m_PresentBodyPosition.y, m_PresentBodyPosition.z,
				m_PresentBodyPosition.roll, m_PresentBodyPosition.pitch, m_PresentBodyPosition.yaw);

		matGtoRF = GetTransformMatrix(m_PresentRightFootPosition.x, m_PresentRightFootPosition.y, m_PresentRightFootPosition.z,
				m_PresentRightFootPosition.roll, m_PresentRightFootPosition.pitch, m_PresentRightFootPosition.yaw);

		matGtoLF = GetTransformMatrix(m_PresentLeftFootPosition.x, m_PresentLeftFootPosition.y, m_PresentLeftFootPosition.z,
				m_PresentLeftFootPosition.roll, m_PresentLeftFootPosition.pitch, m_PresentLeftFootPosition.yaw);


		matCOBtoG = GetTransformMatrixInverse(matGtoCOB);

		matRHtoRF = matRHtoCOB*matCOBtoG*matGtoRF;
		matLHtoLF = matLHtoCOB*matCOBtoG*matGtoLF;

		epr = GetPose3DfromTransformMatrix(matRHtoRF);
		epl = GetPose3DfromTransformMatrix(matLHtoLF);

		//Stabilizer Start
		//Balancing Algorithm
		epr_for_balance.x    = 0; epr_for_balance.y     = 0; epr_for_balance.z   = 0;
		epr_for_balance.roll = 0; epr_for_balance.pitch = 0; epr_for_balance.yaw = 0;
		epl_for_balance.x    = 0; epl_for_balance.y     = 0; epl_for_balance.z   = 0;
		epl_for_balance.roll = 0; epl_for_balance.pitch = 0; epl_for_balance.yaw = 0;

		//		hip_right_roll_adjustment_deg = 0;
		//		hip_left_roll_adjustment_deg = 0;
		hip_roll_adjustment_deg = 0;
		hip_pitch_adjustment_deg = 0;
		foot_r_roll_adjustment_rad_by_ft = 0;
		foot_r_pitch_adjustment_rad_by_ft = 0;
		foot_l_roll_adjustment_rad_by_ft = 0;
		foot_l_pitch_adjustment_rad_by_ft = 0;

		foot_r_roll_adjustment_rad_by_imu  = 0;
		foot_r_pitch_adjustment_rad_by_imu = 0;
		foot_l_roll_adjustment_rad_by_imu  = 0;
		foot_l_pitch_adjustment_rad_by_imu = 0;

		r_x_adjustment_mm_by_ft = 0; r_y_adjustment_mm_by_ft = 0;
		l_x_adjustment_mm_by_ft = 0; l_y_adjustment_mm_by_ft = 0;
		z_adjustment_mm_by_ft = 0;


		double target_fz_N  = 0;
		int Balancing_Index = m_Balancing_Index;
		double right_roll_dir = 1.0;
		double left_roll_dir = 1.0;


		double right_leg_fx_N  = m_right_ft_scale_factor*(current_right_fx_N  - m_init_right_fx_N);
		double right_leg_fy_N  = m_right_ft_scale_factor*(current_right_fy_N  - m_init_right_fy_N);
		double right_leg_fz_N  = m_right_ft_scale_factor*(current_right_fz_N  - m_init_right_fz_N) - 0.5*9.8;
		double right_leg_Tx_Nm = m_right_ft_scale_factor*(current_right_Tx_Nm - m_init_right_Tx_Nm);
		double right_leg_Ty_Nm = m_right_ft_scale_factor*(current_right_Ty_Nm - m_init_right_Ty_Nm);
		double right_leg_Tz_Nm = m_right_ft_scale_factor*(current_right_Tz_Nm - m_init_right_Tz_Nm);

		double left_leg_fx_N  = m_left_ft_scale_factor*(current_left_fx_N  - m_init_left_fx_N);
		double left_leg_fy_N  = m_left_ft_scale_factor*(current_left_fy_N  - m_init_left_fy_N);
		double left_leg_fz_N  = m_left_ft_scale_factor*(current_left_fz_N  - m_init_left_fz_N) - 0.5*9.8;
		double left_leg_Tx_Nm = m_left_ft_scale_factor*(current_left_Tx_Nm - m_init_left_Tx_Nm);
		double left_leg_Ty_Nm = m_left_ft_scale_factor*(current_left_Ty_Nm - m_init_left_Ty_Nm);
		double left_leg_Tz_Nm = m_left_ft_scale_factor*(current_left_Tz_Nm - m_init_left_Tz_Nm);
		matd mat_right_force, mat_right_torque;
		mat_right_force.resize(4,1);	mat_right_force.fill(0);
		mat_right_torque.resize(4,1);	mat_right_torque.fill(0);
		mat_right_force(0,0) = right_leg_fx_N;
		mat_right_force(1,0) = right_leg_fy_N;
		mat_right_force(2,0) = right_leg_fz_N;
		mat_right_torque(0,0) = right_leg_Tx_Nm;
		mat_right_torque(1,0) = right_leg_Ty_Nm;
		mat_right_torque(2,0) = right_leg_Tz_Nm;


		matd mat_left_force, mat_left_torque;
		mat_left_force.resize(4,1);		mat_left_force.fill(0);
		mat_left_torque.resize(4,1);	mat_left_torque.fill(0);
		mat_left_force(0,0) = left_leg_fx_N;
		mat_left_force(1,0) = left_leg_fy_N;
		mat_left_force(2,0) = left_leg_fz_N;
		mat_left_torque(0,0) = left_leg_Tx_Nm;
		mat_left_torque(1,0) = left_leg_Ty_Nm;
		mat_left_torque(2,0) = left_leg_Tz_Nm;

//		fprintf(fpPreviewControlWalking, "%f %f %f %f %f %f %f %f %f %f %f %f ",
//				MotionStatus::R_LEG_FX ,
//				MotionStatus::R_LEG_FY ,
//				MotionStatus::R_LEG_FZ ,
//				MotionStatus::R_LEG_TX ,
//				MotionStatus::R_LEG_TY ,
//				MotionStatus::R_LEG_TZ ,
//				MotionStatus::L_LEG_FX ,
//				MotionStatus::L_LEG_FY ,
//				MotionStatus::L_LEG_FZ ,
//				MotionStatus::L_LEG_TX ,
//				MotionStatus::L_LEG_TY ,
//				MotionStatus::L_LEG_TZ  );

//				fprintf(fpPreviewControlWalking, "%f %f %f %f %f %f %f %f %f %f %f %f ",
//						right_leg_fx_N  ,
//						right_leg_fy_N  ,
//						right_leg_fz_N  ,
//						right_leg_Tx_Nm ,
//						right_leg_Ty_Nm ,
//						right_leg_Tz_Nm ,
//						left_leg_fx_N  ,
//						left_leg_fy_N  ,
//						left_leg_fz_N  ,
//						left_leg_Tx_Nm ,
//						left_leg_Ty_Nm ,
//						left_leg_Tz_Nm );

		mat_right_force  = matRHtoRF*matRFtoRFT*mat_right_force;
		mat_right_torque = matRHtoRF*matRFtoRFT*mat_right_torque;

		mat_left_force  = matLHtoLF*matLFtoLFT*mat_left_force;
		mat_left_torque = matLHtoLF*matLFtoLFT*mat_left_torque;

		//zmp by ftsensor
		Pose3D poseGtoRF = GetPose3DfromTransformMatrix(matGtoRF);
		Pose3D poseGtoLF = GetPose3DfromTransformMatrix(matGtoLF);


//		fprintf(fpPreviewControlWalking, "%f %f %f %f %f %f %f %f %f %f %f %f ",
//				mat_right_force(0, 0)  ,
//				mat_right_force(1, 0)  ,
//				mat_right_force(2, 0)  ,
//				mat_right_torque(0, 0) ,
//				mat_right_torque(1, 0) ,
//				mat_right_torque(2, 0) ,
//				mat_left_force(0, 0)  ,
//				mat_left_force(1, 0)  ,
//				mat_left_force(2, 0)  ,
//				mat_left_torque(0, 0) ,
//				mat_left_torque(1, 0) ,
//				mat_left_torque(2, 0));


		double x_zmp_mm_by_ft = (poseGtoLF.x * mat_left_force(2,0)*(-1.0) + poseGtoRF.x * mat_right_force(2,0)*(-1.0) + mat_right_torque(1,0)*1000.0 + mat_left_torque(1,0)*1000.0) / (mat_left_force(2,0)*(-1.0) + mat_right_force(2,0)*(-1.0));
		double y_zmp_mm_by_ft = (poseGtoLF.y * mat_left_force(2,0)*(-1.0) + poseGtoRF.y * mat_right_force(2,0)*(-1.0) - mat_right_torque(0,0)*1000.0 - mat_left_torque(0,0)*1000.0) / (mat_left_force(2,0)*(-1.0) + mat_right_force(2,0)*(-1.0));

//		fprintf(fpPreviewControlWalking, "%f %f \n",
//				x_zmp_mm_by_ft  ,
//				y_zmp_mm_by_ft  );


		double gyro_roll_rad_per_sec  = current_gyro_roll_rad_per_sec;
		double gyro_pitch_rad_per_sec =  current_gyro_pitch_rad_per_sec;

		double cut_off_freq = 10.0;
		double _time_unit = 0.008;
		double alpha = 2.0*3.141592653589793*cut_off_freq*_time_unit/(1.0 + 2.0*3.141592653589793*cut_off_freq*_time_unit);
//		alpha = 1.0;

		m_gyro_roll_rad_per_sec  = gyro_roll_rad_per_sec*alpha  + (1.0 - alpha)*m_gyro_roll_rad_per_sec;
		m_gyro_pitch_rad_per_sec = gyro_pitch_rad_per_sec*alpha + (1.0 - alpha)*m_gyro_pitch_rad_per_sec;

		gyro_roll_rad_per_sec  = m_gyro_roll_rad_per_sec;
		gyro_pitch_rad_per_sec = m_gyro_pitch_rad_per_sec;

		double gyro_pitch_error_rad_per_sec = -gyro_pitch_rad_per_sec;
		double gyro_roll_error_rad_per_sec = -gyro_roll_rad_per_sec;

		double iu_roll_rad = current_imu_roll_rad;
		double iu_pitch_rad = current_imu_pitch_rad;

		//double iu_roll_error_rad = m_iu_roll_init_rad - iu_roll_rad;
		//double iu_pitch_error_rad = m_iu_pitch_init_rad - iu_pitch_rad;

		double iu_roll_error_rad = 0.0 - iu_roll_rad;
		double iu_pitch_error_rad = 0.0 - iu_pitch_rad;


		if(BALANCE_ENABLE)
		{
			cob_x_adjustment_mm = (IMU_GYRO_GAIN_RATIO * iu_pitch_error_rad + gyro_pitch_error_rad_per_sec) * BALANCE_X_GAIN;
			cob_y_adjustment_mm = (IMU_GYRO_GAIN_RATIO * iu_roll_error_rad  + gyro_roll_error_rad_per_sec) * BALANCE_Y_GAIN;
			cob_z_adjustment_mm = (IMU_GYRO_GAIN_RATIO * iu_roll_error_rad  + gyro_roll_error_rad_per_sec) * BALANCE_Z_GAIN*0.0;

			//switch(Balancing_Index){
			switch(Balancing_Index){
			case 0:
				if(DEBUG_PRINT)
					fprintf(stderr, "DSP : START\n");
				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_landing_detection_time_sec = 0;
				target_fz_N = m_left_dsp_fz_N - m_right_dsp_fz_N;
				right_roll_dir = left_roll_dir = 1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				right_leg_fz_N = m_right_dsp_fz_N;
				//				left_leg_fz_N = m_left_dsp_fz_N*0.9;
				break;
			case 1:
				if(DEBUG_PRINT)
					fprintf(stderr, "DSP : R--O->L\n");
				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				target_fz_N = m_left_dsp_fz_N - m_right_dsp_fz_N;
				right_roll_dir = left_roll_dir = 1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				right_leg_fz_N = m_right_dsp_fz_N;
				//				left_leg_fz_N = m_left_dsp_fz_N*0.9;
				break;
			case 2:
				if(DEBUG_PRINT)
					fprintf(stderr, "SSP : L_BALANCING1\n");
				foot_r_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
				foot_r_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_landing_detection_time_sec = 0;
				target_fz_N = m_left_ssp_fz_N;
				right_roll_dir = -1.0;
				left_roll_dir = 1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				right_leg_fz_N = 0;
				//				left_leg_fz_N = m_left_ssp_fz_N*0.9;
				break;
			case 3:
				if(DEBUG_PRINT)
					fprintf(stderr, "SSP : L_BALANCING2\n");
				foot_r_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
				foot_r_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				target_fz_N = m_left_ssp_fz_N;
				//				right_leg_fz_N = 0;
				//				left_leg_fz_N = m_left_ssp_fz_N*0.9;
				right_roll_dir = -1.0;
				left_roll_dir = 1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				break;
			case 4:
				if(DEBUG_PRINT)
					fprintf(stderr, "DSP : R--O<-L\n");

				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_landing_detection_time_sec = 0;
				target_fz_N = m_left_dsp_fz_N - m_right_dsp_fz_N;
				//				right_leg_fz_N = m_right_dsp_fz_N;
				//				left_leg_fz_N = m_left_dsp_fz_N*0.9;
				right_roll_dir = 1.0;
				left_roll_dir = 1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				break;
			case 5:
				if(DEBUG_PRINT)
					fprintf(stderr, "DSP : R<-O--L\n");

				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				target_fz_N = m_left_dsp_fz_N - m_right_dsp_fz_N;
				//				right_leg_fz_N = m_right_dsp_fz_N;
				//				left_leg_fz_N = m_left_dsp_fz_N*0.9;
				right_roll_dir = 1.0;
				left_roll_dir = 1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				break;
			case 6:
				if(DEBUG_PRINT)
					fprintf(stderr, "SSP : R_BALANCING1\n");
				foot_l_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
				foot_l_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_landing_detection_time_sec = 0;
				target_fz_N = -m_right_ssp_fz_N;
				right_roll_dir = 1.0;
				left_roll_dir = -1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				right_leg_fz_N = m_right_ssp_fz_N;
				//				left_leg_fz_N = 0;
				break;
			case 7:
				if(DEBUG_PRINT)
					fprintf(stderr, "SSP : R_BALANCING2\n");
				foot_l_roll_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_roll_error_rad;
				foot_l_pitch_landing_offset_rad = FOOT_LANDING_OFFSET_GAIN*iu_pitch_error_rad;
				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + 1.0*(IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				target_fz_N =  -m_right_ssp_fz_N;
				right_roll_dir = 1.0;
				left_roll_dir = -1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				right_leg_fz_N = m_right_ssp_fz_N;
				//				left_leg_fz_N = 0;
				break;
			case 8:
				if(DEBUG_PRINT)
					fprintf(stderr, "DSP : R->O--L");

				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				target_fz_N = m_left_dsp_fz_N - m_right_dsp_fz_N;
				right_roll_dir = 1.0;
				left_roll_dir =  1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				right_leg_fz_N = m_right_dsp_fz_N;
				//				left_leg_fz_N = m_left_dsp_fz_N*0.9;
				break;
			case 9:
				if(DEBUG_PRINT)
					fprintf(stderr, "DSP : END");

				foot_r_roll_adjustment_rad  = foot_r_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_r_pitch_adjustment_rad = foot_r_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				foot_l_roll_adjustment_rad  = foot_l_roll_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_roll_error_rad+gyro_roll_error_rad_per_sec) * BALANCE_ROLL_GAIN;
				foot_l_pitch_adjustment_rad = foot_l_pitch_landing_offset_rad + (IMU_GYRO_GAIN_RATIO*iu_pitch_error_rad+gyro_pitch_error_rad_per_sec) * BALANCE_PITCH_GAIN;
				target_fz_N = m_left_dsp_fz_N - m_right_dsp_fz_N;
				right_roll_dir = 1.0;
				left_roll_dir =  1.0;
				//				hip_right_roll_adjustment_deg = GetHipRightRollDampingControllerOutput(0, right_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				hip_left_roll_adjustment_deg  = GetHipLeftRollDampingControllerOutput( 0,  left_roll_dir*iu_roll_rad * 180.0 / PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
				//				right_leg_fz_N = m_right_dsp_fz_N;
				//				left_leg_fz_N = m_left_dsp_fz_N*0.9;
				break;
			default:
				break;
			}

			bool IsDSP = false;

			double r_target_fz_N = m_right_dsp_fz_N;
			double l_target_fz_N = m_left_dsp_fz_N;

			if( (m_Balancing_Index == 0) ||
					(m_Balancing_Index == 1) ||
					(m_Balancing_Index == 4) ||
					(m_Balancing_Index == 5) ||
					(m_Balancing_Index == 8) ||
					(m_Balancing_Index == 9) )
			{
				IsDSP = true;
			}
			else
				IsDSP = false;

			if(IsDSP)
			{
				if( (m_Balancing_Index == 0) ||
						(m_Balancing_Index == 9) ) {
					r_target_fz_N = m_right_dsp_fz_N;
					l_target_fz_N = m_left_dsp_fz_N;
				}
				else {
					double l_target_fz_N_amp = 0;
					double l_target_fz_N_amp_shift = 0;

					l_target_fz_N = wsigmoid(m_WalkingTime - TIME_UNIT, m_Left_Fz_Sigmoid_EndTime -  m_Left_Fz_Sigmoid_StartTime, m_Left_Fz_Sigmoid_StartTime, m_Left_Fz_Sigmoid_Target - m_Left_Fz_Sigmoid_Shift, m_Left_Fz_Sigmoid_Shift, 1.0, 1.0);
					r_target_fz_N = m_left_ssp_fz_N - l_target_fz_N;
				}

			}
			else {
				if( (m_Balancing_Index == 2) || (m_Balancing_Index == 3) ) {
					r_target_fz_N = 0;
					l_target_fz_N = m_left_ssp_fz_N;
				}
				else {
					r_target_fz_N = m_right_ssp_fz_N;
					l_target_fz_N = 0;
				}
			}

			r_x_adjustment_mm_by_ft = GetRightXDampingControlOutput(0.0, mat_right_force.coeff(0,0), BALANCE_X_TIME_CONSTANT, BALANCE_X_GAIN_BY_FT*MMtoM);
			r_y_adjustment_mm_by_ft = GetRightYDampingControlOutput(0.0, mat_right_force.coeff(1,0), BALANCE_Y_TIME_CONSTANT, BALANCE_Y_GAIN_BY_FT*MMtoM);
			l_x_adjustment_mm_by_ft = GetLeftXDampingControlOutput(0.0, mat_left_force.coeff(0,0), BALANCE_X_TIME_CONSTANT, BALANCE_X_GAIN_BY_FT*MMtoM);
			l_y_adjustment_mm_by_ft = GetLeftYDampingControlOutput(0.0, mat_left_force.coeff(1,0), BALANCE_Y_TIME_CONSTANT, BALANCE_Y_GAIN_BY_FT*MMtoM);


			hip_roll_adjustment_deg = GetHipRollDampingControllerOutput(0, iu_roll_rad*180.0/M_PI, BALANCE_HIP_ROLL_TIME_CONSTANT, BALANCE_HIP_ROLL_GAIN);
			hip_pitch_adjustment_deg = GetHipPitchDampingControllerOutput(HIP_PITCH_OFFSET, iu_pitch_rad*180.0/M_PI, BALANCE_HIP_PITCH_TIME_CONSTANT, BALANCE_HIP_PITCH_GAIN);

			foot_r_roll_adjustment_rad_by_imu  = GetAnkleRollDampingControllerOutput(m_PresentBodyPosition.roll, iu_roll_rad,  BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU,  BALANCE_ANKLE_ROLL_GAIN_BY_IMU);
			foot_r_pitch_adjustment_rad_by_imu = GetAnklePitchDampingControllerOutput(m_PresentBodyPosition.pitch + HIP_PITCH_OFFSET*M_PI/180.0, iu_pitch_rad, BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU, BALANCE_ANKLE_PITCH_GAIN_BY_IMU);


			foot_l_roll_adjustment_rad_by_imu = foot_r_roll_adjustment_rad_by_imu;
			foot_l_pitch_adjustment_rad_by_imu = foot_r_pitch_adjustment_rad_by_imu;

			foot_r_roll_adjustment_rad_by_ft  = GetRightAnkleRollMomentDampingControllerOutput( 0.0, mat_right_torque.coeff(0,0), BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT,  BALANCE_RIGHT_ROLL_GAIN_BY_FT);
			foot_r_pitch_adjustment_rad_by_ft = GetRightAnklePitchMomentDampingControllerOutput(0.0, mat_right_torque.coeff(1,0), BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT, BALANCE_RIGHT_PITCH_GAIN_BY_FT);
			foot_l_roll_adjustment_rad_by_ft  = GetLeftAnkleRollMomentDampingControllerOutput(  0.0, mat_left_torque.coeff(0,0),  BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT,  BALANCE_LEFT_ROLL_GAIN_BY_FT);
			foot_l_pitch_adjustment_rad_by_ft = GetLeftAnklePitchMomentDampingControllerOutput( 0.0, mat_left_torque.coeff(1,0),  BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT, BALANCE_LEFT_PITCH_GAIN_BY_FT);
			//z_adjustment_mm_by_ft = GetZDampingControllOutput(target_fz_N, mat_left_force(2,0) - mat_right_force(2,0), BALANCE_Z_TIME_CONSTANT, BALANCE_Z_GAIN_BY_FT);
			z_adjustment_mm_by_ft = GetZDampingControllOutput(l_target_fz_N - r_target_fz_N, mat_left_force.coeff(2,0) - mat_right_force.coeff(2,0), BALANCE_Z_TIME_CONSTANT, BALANCE_Z_GAIN_BY_FT*MMtoM);
			//fprintf(fpBalance, "%f %f %f %f\n", l_target_fz_N, r_target_fz_N, zmp_ref_x_at_this_time, zmp_ref_y_at_this_time);


			//fprintf(fpPreviewControlWalking, "%f %f %f %f\n", l_target_fz_N, r_target_fz_N, zmp_ref_x_at_this_time, zmp_ref_y_at_this_time);


			if(DEBUG_PRINT)
				fprintf(stderr, " : %f %f %f %f %f %f %f %f\n", cob_x_adjustment_mm, cob_y_adjustment_mm, foot_r_roll_adjustment_rad*180.0/M_PI, foot_r_pitch_adjustment_rad*180.0/M_PI, foot_l_roll_adjustment_rad*180.0/M_PI, foot_l_pitch_adjustment_rad*180.0/M_PI,  right_leg_fz_N,  left_leg_fz_N);

			matd matBlanceRotationIMU = GetOrientationMatrix(foot_r_roll_adjustment_rad_by_imu + foot_r_roll_adjustment_rad, foot_r_pitch_adjustment_rad_by_imu + foot_r_pitch_adjustment_rad, 0.0);
			matd epr_xy, epl_xy;
			matd _COBtoRF = matCOBtoG*matGtoRF;
			matd _COBtoLF = matCOBtoG*matGtoLF;
			epr_xy.resize(4,1); epr_xy.fill(0);
			epl_xy.resize(4,1); epl_xy.fill(0);
			epr_xy(0,0) = _COBtoRF(0, 3);		epr_xy(1,0) = _COBtoRF(1, 3); epr_xy(3, 0) = 1.0;
			epl_xy(0,0) = _COBtoLF(0, 3);		epl_xy(1,0) = _COBtoLF(1, 3); epl_xy(3, 0) = 1.0;
			epr_xy = matBlanceRotationIMU*epr_xy;
			epl_xy = matBlanceRotationIMU*epl_xy;


			epr_for_balance.x -= (cob_x_adjustment_mm);
			epr_for_balance.y -= (cob_y_adjustment_mm);
			epr_for_balance.z += (cob_z_adjustment_mm + z_adjustment_mm_by_ft*0.5*1.0 + epr_xy(2,0));

			epr_for_balance.x += r_x_adjustment_mm_by_ft*1.0;
			epr_for_balance.y += r_y_adjustment_mm_by_ft*1.0;

			epr_for_balance.roll += (foot_r_roll_adjustment_rad + 1.0*foot_r_roll_adjustment_rad_by_ft + foot_r_roll_adjustment_rad_by_imu);
			epr_for_balance.pitch += (foot_r_pitch_adjustment_rad + 1.0*foot_r_pitch_adjustment_rad_by_ft + foot_r_pitch_adjustment_rad_by_imu);


			epl_for_balance.x -= (cob_x_adjustment_mm);
			epl_for_balance.y -= (cob_y_adjustment_mm);
			epl_for_balance.z += (-cob_z_adjustment_mm - z_adjustment_mm_by_ft*0.5*1.0 + epl_xy(2,0));

			epl_for_balance.x += l_x_adjustment_mm_by_ft*1.0;
			epl_for_balance.y += l_y_adjustment_mm_by_ft*1.0;

			epl_for_balance.roll += (foot_l_roll_adjustment_rad + 1.0*foot_l_roll_adjustment_rad_by_ft + foot_l_roll_adjustment_rad_by_imu);
			epl_for_balance.pitch += (foot_l_pitch_adjustment_rad + 1.0*foot_l_pitch_adjustment_rad_by_ft + foot_l_pitch_adjustment_rad_by_imu);



			epr_for_balance.x += 0*epr_adjustment_by_axis_controller.x;
			epr_for_balance.y += 0*epr_adjustment_by_axis_controller.y;
			//epr.z += epr_adjustment_by_axis_controller.z;
			epr_for_balance.roll -= 0*epr_adjustment_by_axis_controller.roll;
			epr_for_balance.pitch += 0*epr_adjustment_by_axis_controller.pitch;
			//epr.yaw += epr_adjustment_by_axis_controller.yaw;

			epl_for_balance.x += 0*epl_adjustment_by_axis_controller.x;
			epl_for_balance.y += 0*epl_adjustment_by_axis_controller.y;
			//epl.z += epl_adjustment_by_axis_controller.z;
			epl_for_balance.roll -= 0*epl_adjustment_by_axis_controller.roll;
			epl_for_balance.pitch += 0*epl_adjustment_by_axis_controller.pitch;
			//epl.yaw += epl_adjustment_by_axis_controller.yaw;




			hip_roll_adjustment_deg = sign(hip_roll_adjustment_deg)*fmin(fabs(hip_roll_adjustment_deg), 5.0);
			hip_pitch_adjustment_deg = sign(hip_pitch_adjustment_deg)*fmin(fabs(hip_pitch_adjustment_deg), 5.0);
			epr_for_balance.x 	  = sign(epr_for_balance.x    )*fmin(fabs(epr_for_balance.x),     X_ADJUSTMENT_ABS_MAX_MM*MMtoM);
			epr_for_balance.y 	  = sign(epr_for_balance.y    )*fmin(fabs(epr_for_balance.y),     Y_ADJUSTMENT_ABS_MAX_MM*MMtoM);
			epr_for_balance.z     = sign(epr_for_balance.z    )*fmin(fabs(epr_for_balance.z),     Z_ADJUSTMENT_ABS_MAX_MM*MMtoM);
			epr_for_balance.roll  = sign(epr_for_balance.roll )*fmin(fabs(epr_for_balance.roll),  ROLL_ADJUSTMENT_ABS_MAX_RAD);
			epr_for_balance.pitch = sign(epr_for_balance.pitch)*fmin(fabs(epr_for_balance.pitch), PITCH_ADJUSTMENT_ABS_MAX_RAD);
			epr_for_balance.yaw   = 0;
			epl_for_balance.x 	  = sign(epl_for_balance.x    )*fmin(fabs(epl_for_balance.x),     X_ADJUSTMENT_ABS_MAX_MM*MMtoM);
			epl_for_balance.y 	  = sign(epl_for_balance.y    )*fmin(fabs(epl_for_balance.y),     Y_ADJUSTMENT_ABS_MAX_MM*MMtoM);
			epl_for_balance.z     = sign(epl_for_balance.z    )*fmin(fabs(epl_for_balance.z),     Z_ADJUSTMENT_ABS_MAX_MM*MMtoM);
			epl_for_balance.roll  = sign(epl_for_balance.roll )*fmin(fabs(epl_for_balance.roll),  ROLL_ADJUSTMENT_ABS_MAX_RAD);
			epl_for_balance.pitch = sign(epl_for_balance.pitch)*fmin(fabs(epl_for_balance.pitch), PITCH_ADJUSTMENT_ABS_MAX_RAD);
			epl_for_balance.yaw   = 0;


			//printf("%f %f %f %f\n", zmp_ref_y_at_this_time, l_target_fz_N, r_target_fz_N, target_fz_N);
//			printf("y_zmp                              : %f\n", zmp_ref_y_at_this_time);
//			printf("l_target_fz_N                      : %f\n", l_target_fz_N);
//			printf("r_target_fz_N                      : %f\n", r_target_fz_N);
//			printf("target_fz_N                        : %f\n", target_fz_N);
//			printf("mat_right_force.coeff(0,0)         : %f\n", mat_right_force.coeff(0,0));
//			printf("mat_right_force.coeff(1,0)         : %f\n", mat_right_force.coeff(1,0));
//			printf("mat_right_force.coeff(2,0)         : %f\n", mat_right_force.coeff(2,0));
//			printf("mat_right_torque.coeff(0,0)        : %f\n", mat_right_torque.coeff(0,0));
//			printf("mat_right_torque.coeff(1,0)        : %f\n", mat_right_torque.coeff(1,0));
//			printf("foot_r_roll_adjustment_rad         : %f\n", foot_r_roll_adjustment_rad);
//			printf("foot_r_pitch_adjustment_rad        : %f\n", foot_r_pitch_adjustment_rad);
//			printf("foot_r_roll_adjustment_rad_by_imu  : %f\n", foot_r_roll_adjustment_rad_by_imu);
//			printf("foot_r_pitch_adjustment_rad_by_imu : %f\n", foot_r_pitch_adjustment_rad_by_imu);
//			printf("r_x_adjustment_mm_by_ft            : %f\n", r_x_adjustment_mm_by_ft);
//			printf("r_y_adjustment_mm_by_ft            : %f\n", r_y_adjustment_mm_by_ft);
//			printf("r_z_adjustment_mm_by_ft            : %f\n", z_adjustment_mm_by_ft*0.5*1.0);
//			printf("foot_r_roll_adjustment_rad_by_ft   : %f\n", foot_r_roll_adjustment_rad_by_ft);
//			printf("foot_r_pitch_adjustment_rad_by_ft  : %f\n", foot_r_pitch_adjustment_rad_by_ft);
//			printf("epr_for_balance.roll               : %f\n", epr_for_balance.roll);
//			printf("epr_for_balance.pitch              : %f\n", epr_for_balance.pitch);
//			printf("\n");
//			printf("mat_left_force.coeff(0,0)         : %f\n", mat_left_force.coeff(0,0));
//			printf("mat_left_force.coeff(1,0)         : %f\n", mat_left_force.coeff(1,0));
//			printf("mat_left_force.coeff(2,0)         : %f\n", mat_left_force.coeff(2,0));
//			printf("mat_left_torque.coeff(0,0)        : %f\n", mat_left_torque.coeff(0,0));
//			printf("mat_left_torque.coeff(1,0)        : %f\n", mat_left_torque.coeff(1,0));
//			printf("foot_l_roll_adjustment_rad         : %f\n", foot_l_roll_adjustment_rad);
//			printf("foot_l_pitch_adjustment_rad        : %f\n", foot_l_pitch_adjustment_rad);
//			printf("foot_l_roll_adjustment_rad_by_imu  : %f\n", foot_l_roll_adjustment_rad_by_imu);
//			printf("foot_l_pitch_adjustment_rad_by_imu : %f\n", foot_l_pitch_adjustment_rad_by_imu);
//			printf("l_x_adjustment_mm_by_ft            : %f\n", l_x_adjustment_mm_by_ft);
//			printf("l_y_adjustment_mm_by_ft            : %f\n", l_y_adjustment_mm_by_ft);
//			printf("l_z_adjustment_mm_by_ft            : %f\n", -z_adjustment_mm_by_ft*0.5*1.0);
//			printf("foot_l_roll_adjustment_rad_by_ft   : %f\n", foot_l_roll_adjustment_rad_by_ft);
//			printf("foot_l_pitch_adjustment_rad_by_ft  : %f\n", foot_l_pitch_adjustment_rad_by_ft);
//			printf("epl_for_balance.roll               : %f\n", epl_for_balance.roll);
//			printf("epl_for_balance.pitch              : %f\n", epl_for_balance.pitch);
//			printf("--------------------------------------------------------------\n");

		}
		//Stabilizer End


		vecd r_foot_rotation = GetEulerRollPitchYaw(GetOrientationMatrix(epr_for_balance.roll, epr_for_balance.pitch, epr_for_balance.yaw)*GetOrientationMatrix(epr.roll, epr.pitch, epr.yaw));
		vecd l_foot_rotation  = GetEulerRollPitchYaw(GetOrientationMatrix(epl_for_balance.roll, epl_for_balance.pitch, epl_for_balance.yaw)*GetOrientationMatrix(epl.roll, epl.pitch, epl.yaw));

//		matd matCOB_toCOB = GetTransformMatrix(0, 0, 0, -1.0*body_roll_swap, 0, 0);
//
//		matRHtoRF = matRHtoCOB*matCOB_toCOB*matCOBtoG*matGtoRF;
//		matLHtoLF = matLHtoCOB*matCOB_toCOB*matCOBtoG*matGtoLF;

//		epr = GetPose3DfromTransformMatrix(matRHtoRF);
//		epl = GetPose3DfromTransformMatrix(matLHtoLF);


		epr.x     += epr_for_balance.x - COB_X_MANUAL_ADJUSTMENT_M;		epl.x     += epl_for_balance.x - COB_X_MANUAL_ADJUSTMENT_M;
		epr.y     += epr_for_balance.y - COB_Y_MANUAL_ADJUSTMENT_M;		epl.y     += epl_for_balance.y - COB_Y_MANUAL_ADJUSTMENT_M;
		epr.z     += epr_for_balance.z - COB_Z_MANUAL_ADJUSTMENT_M;		epl.z     += epl_for_balance.z - COB_Z_MANUAL_ADJUSTMENT_M;
		epr.roll   = r_foot_rotation(0);	epl.roll   = l_foot_rotation(0);
		epr.pitch  = r_foot_rotation(1);	epl.pitch  = l_foot_rotation(1);
		epr.yaw    = r_foot_rotation(2);	epl.yaw    = l_foot_rotation(2);



		if((epr.yaw > 30.0*M_PI/180.0) || (epl.yaw < -30.0*M_PI/180.0) ) {
			printf("yawyaw\n");
			return;
		}

		if((epr.yaw < -30.0*M_PI/180.0) || (epl.yaw > 30.0*M_PI/180.0) ) {
			printf("yawyaw2\n");
			return;
		}

//		double angle[12] = { 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,};
//		if(computeIK(&angle[0], epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw) == false)	{
//			printf("IK not Solved EPR : %f %f %f %f %f %f\n", epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw);
//			return;
//		}
//
//		if(computeIK(&angle[6], epl.x, epl.y, epl.z, epl.roll, epl.pitch, epl.yaw) == false)	{
//			printf("IK not Solved EPL : %f %f %f %f %f %f\n", epl.x, epr.y, epl.z, epl.roll, epl.pitch, epl.yaw);
//			return;
//		}
//
//		m_OutAngleRad[0]  = angle[0];
//		m_OutAngleRad[1]  = angle[1];
//		m_OutAngleRad[2]  = angle[2];
//		m_OutAngleRad[3]  = angle[3];
//		m_OutAngleRad[4]  = angle[4];
//		m_OutAngleRad[5]  = angle[5];
//		m_OutAngleRad[6]  = angle[6];
//		m_OutAngleRad[7]  = angle[7];
//		m_OutAngleRad[8]  = angle[8];
//		m_OutAngleRad[9]  = angle[9];
//		m_OutAngleRad[10] = angle[10];
//		m_OutAngleRad[11] = angle[11];
//
//
//
//		for(int idx = 0; idx < 6; idx++)
//		{
//			m_OutAngleDeg[idx] = (double)dir_output[idx]*m_OutAngleRad[idx]*180.0/PI + m_InitAngle[idx];
//			m_OutAngleDeg[idx+6] = (double)dir_output[idx+6]*m_OutAngleRad[idx+6]*180.0/PI + m_InitAngle[idx+6];
//
//			//			double right_joint_speed = ((m_OutAngleDeg[idx] - m_PrevOutAngleDeg[idx])/0.008)/6.0;
//			//			if(fabs(right_joint_speed) > 28.0)
//			//				fprintf(stderr, "right %d th joint exceed nominal speed(%f)\n", idx, right_joint_speed);
//			//
//			//			double left_joint_speed = ((m_OutAngleDeg[idx+6] - m_PrevOutAngleDeg[idx+6])/0.008)/6.0;
//			//			if(fabs(left_joint_speed) > 28.0)
//			//				fprintf(stderr, "left %d th joint exceed nominal speed(%f)\n", idx, left_joint_speed);
//		}

		double angle[12] = { 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,};
		if(thormang3_kd_->InverseKinematicsforRightLeg(&angle[0], epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw) == false)	{
			printf("IK not Solved EPR : %f %f %f %f %f %f\n", epr.x, epr.y, epr.z, epr.roll, epr.pitch, epr.yaw);
			return;
		}

		if(thormang3_kd_->InverseKinematicsforLeftLeg(&angle[6], epl.x, epl.y, epl.z, epl.roll, epl.pitch, epl.yaw) == false)	{
			printf("IK not Solved EPL : %f %f %f %f %f %f\n", epl.x, epr.y, epl.z, epl.roll, epl.pitch, epl.yaw);
			return;
		}

		m_OutAngleRad[0]  = angle[0];
		m_OutAngleRad[1]  = angle[1];
		m_OutAngleRad[2]  = angle[2];
		m_OutAngleRad[3]  = angle[3];
		m_OutAngleRad[4]  = angle[4];
		m_OutAngleRad[5]  = angle[5];
		m_OutAngleRad[6]  = angle[6];
		m_OutAngleRad[7]  = angle[7];
		m_OutAngleRad[8]  = angle[8];
		m_OutAngleRad[9]  = angle[9];
		m_OutAngleRad[10] = angle[10];
		m_OutAngleRad[11] = angle[11];

		m_OutAngleRad[12] = ((double)dir_output[12]*(epr.x - epl.x)*m_ShoulerSwingGain + m_InitAngle[12])*M_PI/180.0;
		m_OutAngleRad[13] = ((double)dir_output[13]*(epl.x - epr.x)*m_ShoulerSwingGain + m_InitAngle[13])*M_PI/180.0;
		m_OutAngleRad[14] = ((double)dir_output[14]*(epr.x - epl.x)*m_ElbowSwingGain   + m_InitAngle[14])*M_PI/180.0;
		m_OutAngleRad[15] = ((double)dir_output[15]*(epl.x - epr.x)*m_ElbowSwingGain   + m_InitAngle[15])*M_PI/180.0;



		//		m_OutAngleValue[1] -= (double)dir_output[1] * (hip_roll_adjustment_deg*1.0 + hip_right_roll_adjustment_deg*0.0) * 250950.0/180.0;
		//		m_OutAngleValue[7] -= (double)dir_output[7] * (hip_roll_adjustment_deg*1.0 + hip_left_roll_adjustment_deg*0.0) * 250950.0/180.0;

		m_OutAngleRad[1] -= (double)dir_output[1] * hip_roll_adjustment_deg * 1.0 * M_PI/180.0;
		m_OutAngleRad[7] -= (double)dir_output[7] * hip_roll_adjustment_deg * 1.0 * M_PI/180.0;

		m_OutAngleRad[2] -= (double)dir_output[2] * (HIP_PITCH_OFFSET + hip_pitch_adjustment_deg) * M_PI/180.0;
		m_OutAngleRad[8] -= (double)dir_output[8] * (HIP_PITCH_OFFSET + hip_pitch_adjustment_deg) * M_PI/180.0;

		m_OutAngleRad[4]  += (double)dir_output[4]  * ANKLE_PITCH_OFFSET * M_PI/180.0;
		m_OutAngleRad[10] += (double)dir_output[10] * ANKLE_PITCH_OFFSET * M_PI/180.0;


		//printf("%f %f %f %f %f\n", hip_roll_adjustment_deg, HIP_PITCH_OFFSET, hip_pitch_adjustment_deg, current_imu_roll_rad, current_imu_pitch_rad);


		if(m_StepData.size() != 0) {
			if(m_StepData[0].PositionData.bMovingFoot == LFootMove)
				m_OutAngleRad[1] = m_OutAngleRad[1] + body_roll_swap;
			else if(m_StepData[0].PositionData.bMovingFoot == RFootMove)
				m_OutAngleRad[1] = m_OutAngleRad[1] - 0.35*body_roll_swap;
		}



		if(m_StepData.size() != 0) {
			if(m_StepData[0].PositionData.bMovingFoot == RFootMove)
				m_OutAngleRad[7] = m_OutAngleRad[7] + body_roll_swap;
			else if(m_StepData[0].PositionData.bMovingFoot == LFootMove)
				m_OutAngleRad[7] = m_OutAngleRad[7] - 0.35*body_roll_swap;
		}

	}
}


double PreviewControlWalking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

double PreviewControlWalking::wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio)
{
	double value = mag_shift, Amplitude = 0.0, sigmoid_distor_gain = 1.0, t = 0.0;
	if ((sigmoid_ratio >= 1.0) && (sigmoid_ratio < 2.0)) {

		if( time >= time_shift+period*(2-sigmoid_ratio)) {
			value = mag_shift + mag;
		}
		else
		{
			t = 2.0*M_PI*(time - time_shift)/(period*(2-sigmoid_ratio));
			sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*(2-sigmoid_ratio));
			Amplitude = mag/(2.0*M_PI);
			value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
		}
	}
	else if( (sigmoid_ratio >= 0.0) && (sigmoid_ratio < 1.0)) {
		if( time <= time_shift+period*(1-sigmoid_ratio))
			value = mag_shift;
		else {
			t = 2.0*M_PI*(time - time_shift-period*(1-sigmoid_ratio))/(period*sigmoid_ratio);
			sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*sigmoid_ratio);
			Amplitude = mag/(2.0*M_PI);
			value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
		}
	}
	else if(( sigmoid_ratio >= 2.0) && ( sigmoid_ratio < 3.0)) {
		double nsigmoid_ratio = sigmoid_ratio - 2.0;
		if(time <= time_shift + period*(1.0-nsigmoid_ratio)*0.5)
			value = mag_shift;
		else if(time >= time_shift + period*(1.0+nsigmoid_ratio)*0.5)
			value = mag + mag_shift;
		else {
			t = 2.0*M_PI*(time - (time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
			sigmoid_distor_gain = distortion_ratio + (1.0-distortion_ratio)*(time-(time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
			Amplitude = mag/(2.0*M_PI);
			value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
		}
	}
	else
		value = mag_shift;

	return value;
}

void PreviewControlWalking::SetFTScaleFactor(double right_ft_scale_factor, double left_ft_scale_factor)
{
	m_right_ft_scale_factor = right_ft_scale_factor;
	m_left_ft_scale_factor = left_ft_scale_factor;
}
