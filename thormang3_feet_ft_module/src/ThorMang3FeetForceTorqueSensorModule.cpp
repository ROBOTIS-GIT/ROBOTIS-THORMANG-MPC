/*
 * ThorMang3FootForceTorqueSensorModule.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: jay
 */

#include "thormang3_feet_ft_module/ThorMang3FeetForceTorqueSensorModule.h"

#define EXT_PORT_DATA_1 "external_port_data_1"
#define EXT_PORT_DATA_2 "external_port_data_2"
#define EXT_PORT_DATA_3 "external_port_data_3"
#define EXT_PORT_DATA_4 "external_port_data_4"

using namespace ROBOTIS;

ThorMang3FeetForceTorqueSensor::ThorMang3FeetForceTorqueSensor()
    : control_cycle_msec_(8)
{

    module_name     = "thormang3_foot_force_torque_sensor_module"; // set unique module name

    thormang3_kd_ = new ROBOTIS::ThorMang3KinematicsDynamics(ROBOTIS::WHOLE_BODY);

    r_foot_ft_air_.resize(6, 1); r_foot_ft_air_.fill(0.0);
    l_foot_ft_air_.resize(6, 1); l_foot_ft_air_.fill(0.0);

    r_foot_ft_gnd_.resize(6, 1); r_foot_ft_gnd_.fill(0.0);
    l_foot_ft_gnd_.resize(6, 1); l_foot_ft_gnd_.fill(0.0);

    r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = 1.0;
    total_mass_ = thormang3_kd_ -> TotalMass(0);

    for(int _idx = 0; _idx < 6; _idx++) {
    	r_foot_ft_current_voltage_[_idx] = 3.3*0.5;
    	l_foot_ft_current_voltage_[_idx] = 3.3*0.5;
    }

	r_foot_fx_raw_N  = r_foot_fy_raw_N  = r_foot_fz_raw_N  = 0;
	r_foot_tx_raw_Nm = r_foot_ty_raw_Nm = r_foot_tz_raw_Nm = 0;
	l_foot_fx_raw_N  = l_foot_fy_raw_N  = l_foot_fz_raw_N  = 0;
	l_foot_tx_raw_Nm = l_foot_ty_raw_Nm = l_foot_tz_raw_Nm = 0;

	r_foot_fx_scaled_N  = r_foot_fy_scaled_N  = r_foot_fz_scaled_N  = 0;
	r_foot_tx_scaled_Nm = r_foot_ty_scaled_Nm = r_foot_tz_scaled_Nm = 0;
	l_foot_fx_scaled_N  = l_foot_fy_scaled_N  = l_foot_fz_scaled_N  = 0;
	l_foot_tx_scaled_Nm = l_foot_ty_scaled_Nm = l_foot_tz_scaled_Nm = 0;


	result["r_foot_fx_raw_N"]	= r_foot_fx_raw_N;
	result["r_foot_fy_raw_N"]	= r_foot_fy_raw_N;
	result["r_foot_fz_raw_N"]	= r_foot_fz_raw_N;
	result["r_foot_tx_raw_Nm"]	= r_foot_tx_raw_Nm;
	result["r_foot_ty_raw_Nm"]	= r_foot_ty_raw_Nm;
	result["r_foot_tz_raw_Nm"]	= r_foot_tz_raw_Nm;

	result["l_foot_fx_raw_N"]	= l_foot_fx_raw_N;
	result["l_foot_fy_raw_N"]	= l_foot_fy_raw_N;
	result["l_foot_fz_raw_N"]	= l_foot_fz_raw_N;
	result["l_foot_tx_raw_Nm"]	= l_foot_tx_raw_Nm;
	result["l_foot_ty_raw_Nm"]	= l_foot_ty_raw_Nm;
	result["l_foot_tz_raw_Nm"]	= l_foot_tz_raw_Nm;


	result["r_foot_fx_scaled_N"]	= r_foot_fx_scaled_N;
	result["r_foot_fy_scaled_N"]	= r_foot_fy_scaled_N;
	result["r_foot_fz_scaled_N"]	= r_foot_fz_scaled_N;
	result["r_foot_tx_scaled_Nm"]	= r_foot_tx_scaled_Nm;
	result["r_foot_ty_scaled_Nm"]	= r_foot_ty_scaled_Nm;
	result["r_foot_tz_scaled_Nm"]	= r_foot_tz_scaled_Nm;

	result["l_foot_fx_scaled_N"]	= l_foot_fx_scaled_N;
	result["l_foot_fy_scaled_N"]	= l_foot_fy_scaled_N;
	result["l_foot_fz_scaled_N"]	= l_foot_fz_scaled_N;
	result["l_foot_tx_scaled_Nm"]	= l_foot_tx_scaled_Nm;
	result["l_foot_ty_scaled_Nm"]	= l_foot_ty_scaled_Nm;
	result["l_foot_tz_scaled_Nm"]	= l_foot_tz_scaled_Nm;


	exist_r_leg_an_r_ = false;
	exist_r_leg_an_p_ = false;
	exist_l_leg_an_r_ = false;
	exist_l_leg_an_p_ = false;


    has_ft_air_		= false;
    has_ft_gnd_		= false;
    ft_command_		= FT_NONE;
    ft_period_		 = 2 * 1000/ control_cycle_msec_;

    ft_get_count_	= 0;
}

ThorMang3FeetForceTorqueSensor::~ThorMang3FeetForceTorqueSensor()
{
    queue_thread_.join();
}

void ThorMang3FeetForceTorqueSensor::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;

    ft_period_		 = 2 * 1000/ control_cycle_msec_;
    ft_get_count_ = ft_period_;


    queue_thread_       = boost::thread(boost::bind(&ThorMang3FeetForceTorqueSensor::QueueThread, this));

    FootForceTorqueSensorInitialize();
}

void ThorMang3FeetForceTorqueSensor::FootForceTorqueSensorInitialize()
{
    ros::NodeHandle _ros_node;


    std::string _foot_ft_data_path  = _ros_node.param<std::string>("ft_data_path", "");
    std::string _ft_calib_data_path = _ros_node.param<std::string>("ft_calibration_data_path", "");

    r_foot_ft_sensor_.Initialize(_foot_ft_data_path, "ft_right_foot", "r_foot_ft_link" , "/robotis/sensor/ft_right_foot/raw", "/robotis/sensor/ft_right_foot/scaled");
    l_foot_ft_sensor_.Initialize(_foot_ft_data_path, "ft_left_foot",  "l_foot_ft_link",  "/robotis/sensor/ft_left_foot/raw",  "/robotis/sensor/ft_left_foot/scaled");


	YAML::Node doc;
//	try
//	{
//		// load yaml
//		doc = YAML::LoadFile(_ft_calib_data_path.c_str());
//	}
//	catch(const std::exception& e)
//	{
//		ROS_ERROR("Fail to load ft_calibration_data yaml file.");
//		return ;
//	}

	doc = YAML::LoadFile(_ft_calib_data_path.c_str());

	std::vector<double> _ft;
	_ft = doc["ft_right_foot_air"].as<std::vector<double> >();
	r_foot_ft_air_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	_ft.clear();
	_ft = doc["ft_right_foot_gnd"].as<std::vector<double> >();
	r_foot_ft_gnd_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	_ft.clear();
	_ft = doc["ft_left_foot_air"].as<std::vector<double> >();
	l_foot_ft_air_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);

	_ft.clear();
	_ft = doc["ft_left_foot_gnd"].as<std::vector<double> >();
	l_foot_ft_gnd_ = Eigen::Map<Eigen::MatrixXd>(_ft.data(), 6, 1);


	double scale = ( r_foot_ft_gnd_.coeff(2, 0) + l_foot_ft_gnd_.coeff(2, 0) - r_foot_ft_air_.coeff(2, 0) - l_foot_ft_air_.coeff(2, 0) ) / (total_mass_ * GRAVITY_ACCELERATION);
	r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = scale;

	r_foot_ft_sensor_.SetScaleParam(r_foot_ft_scale_factor_, r_foot_ft_air_);
	l_foot_ft_sensor_.SetScaleParam(l_foot_ft_scale_factor_, l_foot_ft_air_);
}

void ThorMang3FeetForceTorqueSensor::SaveFTCalibrationData(const std::string &path)
{
    if(has_ft_air_ == false || has_ft_gnd_ == false) return;

    YAML::Emitter _out;

    _out << YAML::BeginMap;

    // air - right
    std::vector<double> _ft_calibration;
    for(int ix = 0; ix < 6; ix++)
        _ft_calibration.push_back(r_foot_ft_air_.coeff(ix, 0));
    _out << YAML::Key << "ft_right_foot_air" << YAML::Value << _ft_calibration;

    // ground - right
    _ft_calibration.clear();
    for(int ix = 0; ix < 6; ix++)
        _ft_calibration.push_back(r_foot_ft_gnd_.coeff(ix, 0));
    _out << YAML::Key << "ft_right_foot_gnd" << YAML::Value << _ft_calibration;

    // air - left
    _ft_calibration.clear();
    for(int ix = 0; ix < 6; ix++)
        _ft_calibration.push_back(l_foot_ft_air_.coeff(ix, 0));
    _out << YAML::Key << "ft_left_foot_air" << YAML::Value << _ft_calibration;

    // ground - left
    _ft_calibration.clear();
    for(int ix = 0; ix < 6; ix++)
        _ft_calibration.push_back(l_foot_ft_gnd_.coeff(ix, 0));
    _out << YAML::Key << "ft_left_foot_gnd" << YAML::Value << _ft_calibration;

    _out << YAML::EndMap;

    // output to file
    std::ofstream fout(path.c_str());
    fout << _out.c_str();

    ROS_INFO("Save FT foot calibration data");
    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Saved FT Calibration Data");
}

void 	ThorMang3FeetForceTorqueSensor::FTSensorCalibrationCommandCallback(const std_msgs::String::ConstPtr& msg)
{
    if( (ft_command_ == FT_NONE) && (ft_period_ == ft_get_count_) )
    {
        std::string _command = msg->data;

        if(_command == "ft_air") {
        	ft_get_count_ = 0;
        	ft_command_ = FT_AIR;
        	PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start measuring FT_AIR");
            has_ft_air_		= false;
            r_foot_ft_air_.fill(0);
            l_foot_ft_air_.fill(0);

        }
        else if(_command == "ft_gnd")
        {
        	ft_get_count_ = 0;
        	ft_command_ = FT_GND;
        	PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start measuring FT_GND");
        	has_ft_gnd_		= false;
        	r_foot_ft_gnd_.fill(0);
        	l_foot_ft_gnd_.fill(0);
        }
        else if(_command == "ft_apply")
        {
        	if(has_ft_air_ && has_ft_gnd_)
        	{
        		double scale = 1.0;
        		scale = ( r_foot_ft_gnd_.coeff(2, 0) + l_foot_ft_gnd_.coeff(2, 0) - r_foot_ft_air_.coeff(2, 0) - l_foot_ft_air_.coeff(2, 0) ) / (total_mass_ * GRAVITY_ACCELERATION);
        		r_foot_ft_scale_factor_ = l_foot_ft_scale_factor_ = scale;

        		PRINT_VAR(r_foot_ft_scale_factor_);
        		PRINT_VAR(l_foot_ft_scale_factor_);
        		PRINT_MAT(r_foot_ft_air_) ;
        		PRINT_MAT(l_foot_ft_air_) ;
        		r_foot_ft_sensor_.SetScaleParam(r_foot_ft_scale_factor_, r_foot_ft_air_);
        		l_foot_ft_sensor_.SetScaleParam(l_foot_ft_scale_factor_, l_foot_ft_air_);

        		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Applied FT Calibration");
        	}
        	else
        		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "There is no value for calibration");
        }
        else if(_command == "ft_save") {
            ros::NodeHandle _ros_node;
        	std::string _ft_calib_data_path = _ros_node.param<std::string>("ft_calibration_data_path", "");
        	SaveFTCalibrationData(_ft_calib_data_path);
        }
    }
    else
        ROS_INFO("previous task is alive");
}

void ThorMang3FeetForceTorqueSensor::PublishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "FeetFT";
    _status.status_msg = msg;

    thormang3_foot_ft_status_pub_.publish(_status);
}

void ThorMang3FeetForceTorqueSensor::PublishBothFTData(int type, Eigen::MatrixXd& ft_right, Eigen::MatrixXd& ft_left)
{
	thormang3_feet_ft_module_msgs::BothWrench _both_wrench_msg;

	if(type == FT_AIR)
		_both_wrench_msg.name = "ft_air";
	else if(type == FT_GND)
		_both_wrench_msg.name = "ft_gnd";
	else
		return;

	_both_wrench_msg.right.force.x  = ft_right.coeff(0,0);
	_both_wrench_msg.right.force.y  = ft_right.coeff(1,0);
	_both_wrench_msg.right.force.z  = ft_right.coeff(2,0);
	_both_wrench_msg.right.torque.x = ft_right.coeff(3,0);
	_both_wrench_msg.right.torque.y = ft_right.coeff(4,0);
	_both_wrench_msg.right.torque.z = ft_right.coeff(5,0);

	_both_wrench_msg.left.force.x  = ft_left.coeff(0,0);
	_both_wrench_msg.left.force.y  = ft_left.coeff(1,0);
	_both_wrench_msg.left.force.z  = ft_left.coeff(2,0);
	_both_wrench_msg.left.torque.x = ft_left.coeff(3,0);
	_both_wrench_msg.left.torque.y = ft_left.coeff(4,0);
	_both_wrench_msg.left.torque.z = ft_left.coeff(5,0);

	thormang3_foot_ft_both_ft_pub_.publish(_both_wrench_msg);

}

void ThorMang3FeetForceTorqueSensor::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscriber */
    ros::Subscriber ft_calib_command_sub	= _ros_node.subscribe("/robotis/feet_ft/ft_calib_command",	1, &ThorMang3FeetForceTorqueSensor::FTSensorCalibrationCommandCallback, this);

    /* publisher */
    thormang3_foot_ft_status_pub_	= _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    thormang3_foot_ft_both_ft_pub_	= _ros_node.advertise<thormang3_feet_ft_module_msgs::BothWrench>("/robotis/feet_ft/both_ft_value", 1);


    ros::WallDuration duration(control_cycle_msec_/1000.0);
    while(_ros_node.ok())
      _callback_queue.callAvailable(duration);
}


void ThorMang3FeetForceTorqueSensor::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, Sensor *> sensors)
{
	exist_r_leg_an_r_ = false;
	exist_r_leg_an_p_ = false;
	exist_l_leg_an_r_ = false;
	exist_l_leg_an_p_ = false;

	std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find("r_leg_an_r");


	if(_dxl_it != dxls.end()) {

		r_foot_ft_current_voltage_[0] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
		r_foot_ft_current_voltage_[1] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
		r_foot_ft_current_voltage_[2] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_3])*3.3/4095.0;
		r_foot_ft_current_voltage_[3] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_4])*3.3/4095.0;
		exist_r_leg_an_r_ = true;
	}
	else
		return;


	_dxl_it = dxls.find("r_leg_an_p");
	if(_dxl_it != dxls.end()) {
		r_foot_ft_current_voltage_[4] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
		r_foot_ft_current_voltage_[5] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
		exist_r_leg_an_p_ = true;
	}
	else
		return;

	_dxl_it = dxls.find("l_leg_an_r");
	if(_dxl_it != dxls.end()) {
		l_foot_ft_current_voltage_[0] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
		l_foot_ft_current_voltage_[1] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
		l_foot_ft_current_voltage_[2] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_3])*3.3/4095.0;
		l_foot_ft_current_voltage_[3] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_4])*3.3/4095.0;
		exist_l_leg_an_r_ = true;
	}
	else
		return;

	_dxl_it = dxls.find("l_leg_an_p");
	if(_dxl_it != dxls.end())	{
		l_foot_ft_current_voltage_[4] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_1])*3.3/4095.0;
		l_foot_ft_current_voltage_[5] = (_dxl_it->second->dxl_state->bulk_read_table[EXT_PORT_DATA_2])*3.3/4095.0;
		exist_l_leg_an_p_ = true;
	}
	else
		return;



	if( exist_r_leg_an_r_ && exist_r_leg_an_p_) {

	    r_foot_ft_sensor_.SetCurrentVoltageOutputPublishForceTorque(	r_foot_ft_current_voltage_[0],
	    																r_foot_ft_current_voltage_[1],
	    																r_foot_ft_current_voltage_[2],
	    																r_foot_ft_current_voltage_[3],
	    																r_foot_ft_current_voltage_[4],
	    																r_foot_ft_current_voltage_[5]);

	    r_foot_ft_sensor_.GetCurrentForceTorqueRaw(&r_foot_fx_raw_N,  &r_foot_fy_raw_N,  &r_foot_fz_raw_N,
    											   &r_foot_tx_raw_Nm, &r_foot_ty_raw_Nm, &r_foot_tz_raw_Nm);
	    r_foot_ft_sensor_.GetCurrentForceTorqueScaled(&r_foot_fx_scaled_N,  &r_foot_fy_scaled_N,  &r_foot_fz_scaled_N,
	    											  &r_foot_tx_scaled_Nm, &r_foot_ty_scaled_Nm, &r_foot_tz_scaled_Nm);

		result["r_foot_fx_raw_N"]	= r_foot_fx_raw_N;
		result["r_foot_fy_raw_N"]	= r_foot_fy_raw_N;
		result["r_foot_fz_raw_N"]	= r_foot_fz_raw_N;
		result["r_foot_tx_raw_Nm"]	= r_foot_tx_raw_Nm;
		result["r_foot_ty_raw_Nm"]	= r_foot_ty_raw_Nm;
		result["r_foot_tz_raw_Nm"]	= r_foot_tz_raw_Nm;


		result["r_foot_fx_scaled_N"]	= r_foot_fx_scaled_N;
		result["r_foot_fy_scaled_N"]	= r_foot_fy_scaled_N;
		result["r_foot_fz_scaled_N"]	= r_foot_fz_scaled_N;
		result["r_foot_tx_scaled_Nm"]	= r_foot_tx_scaled_Nm;
		result["r_foot_ty_scaled_Nm"]	= r_foot_ty_scaled_Nm;
		result["r_foot_tz_scaled_Nm"]	= r_foot_tz_scaled_Nm;

	}

	if( exist_l_leg_an_r_ && exist_l_leg_an_p_) {
		l_foot_ft_sensor_.SetCurrentVoltageOutputPublishForceTorque(l_foot_ft_current_voltage_[0],
																	l_foot_ft_current_voltage_[1],
																	l_foot_ft_current_voltage_[2],
																	l_foot_ft_current_voltage_[3],
																	l_foot_ft_current_voltage_[4],
																	l_foot_ft_current_voltage_[5]);

		l_foot_ft_sensor_.GetCurrentForceTorqueRaw(&l_foot_fx_raw_N,  &l_foot_fy_raw_N,  &l_foot_fz_raw_N,
												   &l_foot_tx_raw_Nm, &l_foot_ty_raw_Nm, &l_foot_tz_raw_Nm);
		l_foot_ft_sensor_.GetCurrentForceTorqueScaled(&l_foot_fx_scaled_N,  &l_foot_fy_scaled_N,  &l_foot_fz_scaled_N,
													  &l_foot_tx_scaled_Nm, &l_foot_ty_scaled_Nm, &l_foot_tz_scaled_Nm);


		result["l_foot_fx_raw_N"]	= l_foot_fx_raw_N;
		result["l_foot_fy_raw_N"]	= l_foot_fy_raw_N;
		result["l_foot_fz_raw_N"]	= l_foot_fz_raw_N;
		result["l_foot_tx_raw_Nm"]	= l_foot_tx_raw_Nm;
		result["l_foot_ty_raw_Nm"]	= l_foot_ty_raw_Nm;
		result["l_foot_tz_raw_Nm"]	= l_foot_tz_raw_Nm;

		result["l_foot_fx_scaled_N"]	= l_foot_fx_scaled_N;
		result["l_foot_fy_scaled_N"]	= l_foot_fy_scaled_N;
		result["l_foot_fz_scaled_N"]	= l_foot_fz_scaled_N;
		result["l_foot_tx_scaled_Nm"]	= l_foot_tx_scaled_Nm;
		result["l_foot_ty_scaled_Nm"]	= l_foot_ty_scaled_Nm;
		result["l_foot_tz_scaled_Nm"]	= l_foot_tz_scaled_Nm;
	}


	if(ft_command_ == FT_NONE )
		return;
	else if(ft_command_ == FT_AIR)
	{
		ft_get_count_++;

		r_foot_ft_air_.coeffRef(0, 0) += r_foot_fx_raw_N;
		r_foot_ft_air_.coeffRef(1, 0) += r_foot_fy_raw_N;
		r_foot_ft_air_.coeffRef(2, 0) += r_foot_fz_raw_N;
		r_foot_ft_air_.coeffRef(3, 0) += r_foot_tx_raw_Nm;
		r_foot_ft_air_.coeffRef(4, 0) += r_foot_ty_raw_Nm;
		r_foot_ft_air_.coeffRef(5, 0) += r_foot_tz_raw_Nm;

		l_foot_ft_air_.coeffRef(0, 0) += l_foot_fx_raw_N;
		l_foot_ft_air_.coeffRef(1, 0) += l_foot_fy_raw_N;
		l_foot_ft_air_.coeffRef(2, 0) += l_foot_fz_raw_N;
		l_foot_ft_air_.coeffRef(3, 0) += l_foot_tx_raw_Nm;
		l_foot_ft_air_.coeffRef(4, 0) += l_foot_ty_raw_Nm;
		l_foot_ft_air_.coeffRef(5, 0) += l_foot_tz_raw_Nm;

		if(ft_get_count_ == ft_period_) {
			r_foot_ft_air_  = r_foot_ft_air_ / (double)ft_period_;
			l_foot_ft_air_  = l_foot_ft_air_ / (double)ft_period_;

			has_ft_air_ = true;
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish measuring FT_AIR");
			PublishBothFTData(FT_AIR, r_foot_ft_air_, l_foot_ft_air_);
			ft_command_ = FT_NONE;
		}
	}
	else if(ft_command_ == FT_GND)
	{
		ft_get_count_++;

		r_foot_ft_gnd_.coeffRef(0, 0) += r_foot_fx_raw_N;
		r_foot_ft_gnd_.coeffRef(1, 0) += r_foot_fy_raw_N;
		r_foot_ft_gnd_.coeffRef(2, 0) += r_foot_fz_raw_N;
		r_foot_ft_gnd_.coeffRef(3, 0) += r_foot_tx_raw_Nm;
		r_foot_ft_gnd_.coeffRef(4, 0) += r_foot_ty_raw_Nm;
		r_foot_ft_gnd_.coeffRef(5, 0) += r_foot_tz_raw_Nm;

		l_foot_ft_gnd_.coeffRef(0, 0) += l_foot_fx_raw_N;
		l_foot_ft_gnd_.coeffRef(1, 0) += l_foot_fy_raw_N;
		l_foot_ft_gnd_.coeffRef(2, 0) += l_foot_fz_raw_N;
		l_foot_ft_gnd_.coeffRef(3, 0) += l_foot_tx_raw_Nm;
		l_foot_ft_gnd_.coeffRef(4, 0) += l_foot_ty_raw_Nm;
		l_foot_ft_gnd_.coeffRef(5, 0) += l_foot_tz_raw_Nm;

		if(ft_get_count_ == ft_period_){
			r_foot_ft_gnd_  = r_foot_ft_gnd_ / (double)ft_period_;
			l_foot_ft_gnd_  = l_foot_ft_gnd_ / (double)ft_period_;

			has_ft_gnd_ = true;
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish measuring FT_GND");
			PublishBothFTData(FT_GND, r_foot_ft_gnd_, l_foot_ft_gnd_);

			ft_command_ = FT_NONE;
		}
	}
	else
		return;

}
