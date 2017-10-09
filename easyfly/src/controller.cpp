#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <easyfly/commands.h>
#include <easyfly/pos_ctrl_sp.h>
#include <easyfly/raw_ctrl_sp.h>
#include <easyfly/trj_ctrl_sp.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <easyfly/state_est.h>
#include <easyfly/output.h>
#include "commons.h"

#include <math.h>
#include "../include/Eigen/Eigen/Eigen"
#include "../include/Eigen/Eigen/Geometry"
using namespace Eigen;
#define _USE_MATH_DEFINES //PI

int g_vehicle_num=2;
//int g_joy_num=2;
int g_joy_num=1;

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}
class PID
{
private:
	float m_kp;
	float m_kd;
	float m_ki;
	float m_kpp;
	float m_ff;
	float m_minOutput;
	float m_maxOutput;
	float m_integratorMin;
	float m_integratorMax;
	float m_integral;
	float m_previousError;
	ros::Time m_previousTime;

public:
	PID(
		float kp,
		float kd,
		float ki,
		float kpp,
		float ff,
		float minOutput,
		float maxOutput,
		float integratorMin,
		float integratorMax)
		: m_kp(kp)
		, m_kd(kd)
		, m_ki(ki)
		, m_kpp(kpp)
		, m_ff(ff)
		, m_minOutput(minOutput)
		, m_maxOutput(maxOutput)
		, m_integratorMin(integratorMin)
		, m_integratorMax(integratorMax)
		, m_integral(0)
		, m_previousError(0)
		, m_previousTime(ros::Time::now())
	{
	}

	void reset()
	{
		m_integral = 0;
		m_previousError = 0;
		m_previousTime = ros::Time::now();
	}

	void setIntegral(float integral)
	{
		m_integral = integral;
	}

	float ki() const
	{
		return m_ki;
	}
	float ff() const
	{
		return m_ff;
	}
	float pid_update(float est, float setpt)
	{
		ros::Time time = ros::Time::now();
		float dt = time.toSec() - m_previousTime.toSec();
		float error = setpt - est;
		m_integral += error * dt;
		m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
		float p = m_kp * error;
		float d = 0;
		if (dt > 0){
			d = m_kd * (error - m_previousError) / dt;
		}
		float i = m_ki * m_integral;
		float output = p + d + i;
		m_previousError = error;
		m_previousTime = time;
		return std::max(std::min(output, m_maxOutput), m_minOutput);
	}
	float pp_update(float est, float setpt)
	{
		float error = setpt - est;
		float output = m_kpp * error;
		return output;
	}

};

class Controller
{
private:
	int m_group_index;

	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	PID m_pidYaw;

	int m_flight_state, m_flight_mode;
	ros::Publisher m_outputpub;
	ros::Subscriber m_estsub, m_magneticsub, m_accsub;
	ros::Subscriber m_rawsub, m_possub, m_trjsub;
	ros::Subscriber m_cmdsub;
	struct M_Ctrl
	{
		easyfly::pos_ctrl_sp pos;
		easyfly::raw_ctrl_sp raw;
		easyfly::trj_ctrl_sp trj;
	};
	M_Ctrl m_ctrl;
//	geometry_msgs::Vector3 m_vel_sp, m_att_sp, m_acc_sp;
	Vector3f v_vel_sp, v_acc_sp;
//	float yawrate_sp;
	float m_xh,m_yh;
    float m_pitch_est,m_roll_est; 
	easyfly::state_est m_est;
	easyfly::commands m_cmd;
	easyfly::output m_output;
	std::string tf_prefix;
//	std::string m_worldFrame;
//	std::string m_frame;
//	tf::TransformListener m_listener;


	
//	ros::ServiceServer m_serviceTakeoff;
//	ros::ServiceServer m_serviceLand;
//	float m_thrust;
//	float m_startZ;
public:
	Controller(
//		const std::string& worldFrame,
//		const std::string& frame,
		const ros::NodeHandle& n)
		:
//		: m_worldFrame(worldFrame)
//		, m_frame(frame)
//		, m_pubNav()
//		, m_listener()
		m_pidX(
			get(n, "PIDs/X/kp"),
			get(n, "PIDs/X/kd"),
			get(n, "PIDs/X/ki"),
			get(n, "PIDs/X/kpp"),
			get(n, "PIDs/X/ff"),
			get(n, "PIDs/X/minOutput"),
			get(n, "PIDs/X/maxOutput"),
			get(n, "PIDs/X/integratorMin"),
			get(n, "PIDs/X/integratorMax"))
		,m_pidY(
			get(n, "PIDs/Y/kp"),
			get(n, "PIDs/Y/kd"),
			get(n, "PIDs/Y/ki"),
			get(n, "PIDs/Y/kpp"),
			get(n, "PIDs/Y/ff"),
			get(n, "PIDs/Y/minOutput"),
			get(n, "PIDs/Y/maxOutput"),
			get(n, "PIDs/Y/integratorMin"),
			get(n, "PIDs/Y/integratorMax"))
		,m_pidZ(
			get(n, "PIDs/Z/kp"),
			get(n, "PIDs/Z/kd"),
			get(n, "PIDs/Z/ki"),
			get(n, "PIDs/Z/kpp"),
			get(n, "PIDs/Z/ff"),
			get(n, "PIDs/Z/minOutput"),
			get(n, "PIDs/Z/maxOutput"),
			get(n, "PIDs/Z/integratorMin"),
			get(n, "PIDs/Z/integratorMax"))
		,m_pidYaw(
			get(n, "PIDs/Yaw/kp"),
			get(n, "PIDs/Yaw/kd"),
			get(n, "PIDs/Yaw/ki"),
			get(n, "PIDs/Yaw/kpp"),
			get(n, "PIDs/Yaw/ff"),
			get(n, "PIDs/Yaw/minOutput"),
			get(n, "PIDs/Yaw/maxOutput"),
			get(n, "PIDs/Yaw/integratorMin"),
			get(n, "PIDs/Yaw/integratorMax"))
	{
		ros::NodeHandle nh("~");//~ means private param
		nh.getParam("ctrl_node_num", m_group_index);
	//	ros::param::get("ctrl_node_num", m_group_index);
	//	m_group_index = node_index;
		char msg_name[50];
		sprintf(msg_name,"/vehicle%d/output", m_group_index);
		m_outputpub = nh.advertise<easyfly::output>(msg_name, 1);

		sprintf(msg_name,"/vehicle%d/state_est",m_group_index);
		m_estsub = nh.subscribe<easyfly::state_est>(msg_name,5,&Controller::estCallback, this);

/*
		sprintf(msg_name,"/vehicle%d/magnetic_field",m_group_index);
		m_magneticsub = nh.subscribe<sensor_msgs::MagneticField>(msg_name, 10,boost::bind(&Controller::magPRYCallback,this));
*/

		sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",m_group_index);
		m_rawsub = nh.subscribe<easyfly::raw_ctrl_sp>(msg_name,5,&Controller::rawctrlCallback, this);

		sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",m_group_index);
		m_possub = nh.subscribe<easyfly::pos_ctrl_sp>(msg_name,5,&Controller::posctrlCallback, this);

		sprintf(msg_name,"/vehicle%d/trj_ctrl_sp",m_group_index);
		m_trjsub = nh.subscribe<easyfly::trj_ctrl_sp>(msg_name,5,&Controller::trjctrlCallback, this);
		
		m_cmdsub = nh.subscribe<easyfly::commands>("commands",5,&Controller::cmdCallback, this);
	}
	void run(double frequency)
	{
		ros::NodeHandle node;
		node.getParam("/flight_mode", m_flight_mode);
		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
		ros::spin();
	}
	void iteration(const ros::TimerEvent& e)
	{
		static float time_elapse = 0;
		float dt = e.current_real.toSec() - e.last_real.toSec();
		time_elapse += dt;
		if(m_cmd.cut){
			m_output.att_sp.x = 0;
			m_output.att_sp.y = 0;
			m_output.att_sp.z = 0;
			m_output.throttle = 0;
			m_outputpub.publish(m_output);
		}
		else{
			switch(m_flight_mode){
				case MODE_RAW:{
					m_output.att_sp.x = m_ctrl.raw.raw_att_sp.x;
					m_output.att_sp.y = m_ctrl.raw.raw_att_sp.y;
					m_output.att_sp.z = m_ctrl.raw.raw_att_sp.z;
					m_output.throttle = m_ctrl.raw.throttle;
					m_outputpub.publish(m_output);
				}
				break;
				case MODE_POS:{
					v_vel_sp(0) = m_pidX.pp_update(m_est.pos_est.x, m_ctrl.pos.pos_sp.x);
					v_vel_sp(1) = m_pidY.pp_update(m_est.pos_est.y, m_ctrl.pos.pos_sp.y);
					v_vel_sp(2) = m_pidZ.pp_update(m_est.pos_est.z, m_ctrl.pos.pos_sp.z);
					v_vel_sp(0) += m_ctrl.pos.vel_ff.x * m_pidX.ff();
					v_vel_sp(1) += m_ctrl.pos.vel_ff.y * m_pidY.ff();
					v_vel_sp(2) += m_ctrl.pos.vel_ff.z * m_pidZ.ff();
					v_acc_sp(0) = m_pidX.pid_update(m_est.vel_est.x, v_vel_sp(0));
					v_acc_sp(1) = m_pidY.pid_update(m_est.vel_est.y, v_vel_sp(1));
					v_acc_sp(2) = m_pidZ.pid_update(m_est.vel_est.z, v_vel_sp(2));
					v_acc_sp(2) += (float)GRAVITY / 1000.0;

					float thrust_force = v_acc_sp.norm() * (float)VEHICLE_MASS / 1000.0;
					Vector3f body_z_sp = v_acc_sp / v_acc_sp.norm();
					Vector3f y_c;
					y_c(0) = -sin(m_ctrl.pos.yaw_sp);
					y_c(1) = cos(m_ctrl.pos.yaw_sp);
					y_c(2) = 0;
					Vector3f body_x_sp = y_c.cross(body_z_sp);
					body_x_sp = body_x_sp / body_x_sp.norm();
					Vector3f body_y_sp = body_z_sp.cross(body_x_sp);
					float R_sp[3][3];
					for (int i = 0; i < 3; i++) {
						R_sp[i][0] = body_x_sp(i);
						R_sp[i][1] = body_y_sp(i);
						R_sp[i][2] = body_z_sp(i);
					}
					m_output.att_sp.x = atan2(R_sp[2][1], R_sp[2][2]);
					m_output.att_sp.y = -asin(R_sp[2][0]);
					m_output.throttle = thrust_force * 1000.0 / MAX_THRUST;
					m_output.att_sp.z = m_pidYaw.pp_update(m_est.yaw_est, m_ctrl.pos.yaw_sp);
					m_outputpub.publish(m_output);
				}
				break;
				case MODE_TRJ:{
					
				}
				break;
				default:
				break;
			}//end switch flight mode
		}//end if cut
	}

	/*void accPR(const sensor_msgs::Imu::ConstPtr& msg)
    {

    	m_pitch_est = -(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);//rad
    	m_roll_est = (msg->linear_acceleration.y,msg->linear_acceleration.x,msg->linear_acceleration.z); //rad
    }
    void magPRYCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
    {
    	m_xh = msg->magnetic_field.y*cos(m_roll_est)+msg->magnetic_field.x*sin(m_roll_est)*sin(m_pitch_est)-msg->magnetic_field.z*cos(m_pitch_est)*sin(m_roll_est);		 
    	m_yh = msg->magnetic_field.x*cos(m_pitch_est)+msg->magnetic_field.z*sin(m_pitch_est);
    	m_est.yaw_est = -atan2(m_xh,m_yh)+1.57f;
    	if(m_est.yaw_est < -M_PI )
    		m_est.yaw_est = m_est.yaw_est + 2*M_PI;
    	if(m_est.yaw_est > M_PI )
    		m_est.yaw_est = m_est.yaw_est - 2*M_PI;
    	/*stateAtt->Euler.R = roll;
    	stateAtt->Euler.P = m_pitch_est;
    	stateAtt->Euler.Y = yaw;
    	stateAtt->rate.R = 0;
    	stateAtt->rate.P = 0;
    	stateAtt->rate.Y = 0;
    	euler2quaternion(&stateAtt->Euler, &stateAtt->Q);
    	quaternion2rotation(&stateAtt->Q, &stateAtt->R);*/
    //}

	void estCallback(const easyfly::state_est::ConstPtr& est)
	{
		m_est.pos_est.x = est->pos_est.x;
		m_est.pos_est.y = est->pos_est.y;
		m_est.pos_est.z = est->pos_est.z;
		m_est.vel_est.x = est->vel_est.x;
		m_est.vel_est.y = est->vel_est.y;
		m_est.vel_est.z = est->vel_est.z;
		m_est.yaw_est = est->yaw_est;
	}
	void rawctrlCallback(const easyfly::raw_ctrl_sp::ConstPtr& ctrl)
	{
		m_ctrl.raw.raw_att_sp.x = ctrl->raw_att_sp.x;
		m_ctrl.raw.raw_att_sp.y = ctrl->raw_att_sp.y;
		m_ctrl.raw.raw_att_sp.z = ctrl->raw_att_sp.z;
		m_ctrl.raw.throttle = ctrl->throttle;
	}
	void posctrlCallback(const easyfly::pos_ctrl_sp::ConstPtr& ctrl)
	{
		m_ctrl.pos.pos_sp.x = ctrl->pos_sp.x;
		m_ctrl.pos.pos_sp.y = ctrl->pos_sp.y;
		m_ctrl.pos.pos_sp.z = ctrl->pos_sp.z;
		m_ctrl.pos.vel_ff.x = ctrl->vel_ff.x;
		m_ctrl.pos.vel_ff.y = ctrl->vel_ff.y;
		m_ctrl.pos.vel_ff.z = ctrl->vel_ff.z;
		m_ctrl.pos.yaw_sp = ctrl->yaw_sp;
	}
	void trjctrlCallback(const easyfly::trj_ctrl_sp::ConstPtr& ctrl)
	{
		
	}
	void cmdCallback(const easyfly::commands::ConstPtr& cmd)
	{
		m_cmd.flight_state = cmd->flight_state;
		m_cmd.l_flight_state = cmd->l_flight_state;
		m_cmd.cut = cmd->cut;
	}

};
int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
	char indexc = *argv[1];
	int index = indexc;
	char node_name[50];
	sprintf(node_name, "controller%d", index);
	ros::init(argc, argv, node_name);
	ros::NodeHandle n("~");
//	n.getParam("ctrl_node_num", index);
	n.getParam("/vehicle_num", g_vehicle_num);
	n.getParam("/joy_num", g_joy_num);
	Controller controller(n);
	controller.run(50);
	return 0;
}