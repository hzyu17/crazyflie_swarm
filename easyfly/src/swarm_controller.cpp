#include "ros/ros.h"
#include <stdio.h> //sprintf, FILE* operations
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <easyfly/pos_ctrl_sp.h>
#include <easyfly/raw_ctrl_sp.h>
#include <easyfly/trj_ctrl_sp.h>
#include <easyfly/pos_est.h>
#include <easyfly/att_est.h>
#include <easyfly/commands.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <easyfly/output.h>
#include "commons.h"
//#include <crazyflie_driver/num_vehiclepub.h>
#include "crazyflie_driver/Yaw_est.h"
#include <math.h>
#include "../include/Eigen/Eigen/Eigen"
#include "../include/Eigen/Eigen/Geometry"
#include <fstream>
using namespace Eigen;
using namespace std;
#define _USE_MATH_DEFINES //PI

int g_joy_num=1;

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
	if (!n.hasParam(name)){
		ROS_INFO("No param named%s",name.c_str());
		return 0.0f;
	}
	else{
		double value;
    	n.getParam(name, value);
    	return value;
	}
    
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
	friend class Swarm_Controller;

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

class Swarm_Controller
{
private:
	int m_group_index;
	int m_vehicle_num;
	//PID m_pidX,m_pidY,m_pidZ,m_pidYaw;
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	PID m_pidRoll;
	PID m_pidPitch;
	PID m_pidYaw;
	float m_pidx_kp;
	int m_flight_state, m_flight_mode;
	bool m_is_pos_original;
	//char m_resFname[50];
	struct M_Ctrl
	{
		//easyfly::pos_ctrl_sp pos;
		easyfly::raw_ctrl_sp raw;
		easyfly::trj_ctrl_sp trj;
	};
	M_Ctrl m_ctrl;
	struct M_est_Vecs
	{
		Vector3f m_pos_est, m_vel_est, m_att_est, m_lpos, m_latt, m_cfImuAcc, m_cfImuVel, m_cfImuPos;
	};
	M_est_Vecs m_est_vecs;
	struct M_sp_Vecs
	{
		Vector3f v_vel_sp, v_acc_sp, v_att_sp, v_posctrl_posSp, v_posctrl_velFF, v_posctrl_attSp;
	};
	M_sp_Vecs m_sp_vecs;
	struct M_Pubs
	{
		ros::Publisher m_outputpub, m_pos_estpub;
	};
	M_Pubs m_pubs;
	struct M_subs
	{
		ros::Subscriber m_pos_estpub, m_att_estsub, m_accsub, m_viconsub, m_yawsub;
		ros::Subscriber m_rawsub, m_possub, m_trjsub, cfIMUsub;
		ros::Subscriber m_cmdsub;
	};
	M_subs m_subs;
	struct M_times
	{
		ros::Time m_previousTime, m_latt_time, m_lpos_time; 
	};
	M_times m_times;

	easyfly::commands m_cmd;
	easyfly::output m_output;
	std::string tf_prefix;

public:
	
	Swarm_Controller(
		const ros::NodeHandle& n)
		:m_is_pos_original(true)
		,m_cmd()
		,m_ctrl()
		,m_sp_vecs()
		,m_est_vecs()
		,m_pubs()
		,m_subs()
		,m_times()
		//,m_resFname("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/")
		,m_pidX(
			40.0,
			20.0,
			2.0,
			2.0,
			0.6,
			-10.0,
			10.0,
			-0.1,
			0.1)
		,m_pidY(
			-40.0,
			-20.0,
			-2.0,
			2.0,
			0.6,
			-10.0,
			10.0,
			-0.1,
			0.1)
		,m_pidZ(
			5000.0,
			6000.0,
			3500.0,
			2.0,
			0.5,
			10000.0,
			60000.0,
			-1000.0,
			1000.0)
		,m_pidRoll(
			-200.0,
			-20.0,
			0.0,
			2.0,
			0.6,
			-200.0,
			200.0,
			0.0,
			0.0)
		,m_pidPitch(
			-200.0,
			-20.0,
			0.0,
			2.0,
			0.6,
			-200.0,
			200.0,
			0.0,
			0.0)
		,m_pidYaw(
			-200.0,
			-20.0,
			0.0,
			2.0,
			0.6,
			-200.0,
			200.0,
			0.0,
			0.0)
	{
		ros::NodeHandle nh("~");//~ means private param
		nh.getParam("group_index", m_group_index);

		/*char msg_name[50];
  		num_vehiclesub = nh.subscribe<crazyflie_driver::num_vehiclepub>("/num_vehiclepub",5,&Swarm_Controller::num_veh_Callback, this);*/
		char msg_name[50];
		sprintf(msg_name,"/vehicle%d/output", m_group_index);
		m_pubs.m_outputpub = nh.advertise<easyfly::output>(msg_name, 5);

		sprintf(msg_name,"/vehicle%d/pos_est", m_group_index); 
		m_pubs.m_pos_estpub = nh.advertise<easyfly::pos_est>(msg_name, 5);

		//sprintf(msg_name,"/vicon/crazyflie%d/whole",m_group_index);
		m_subs.m_viconsub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/one_cf/cf",5,&Swarm_Controller::vicon_Callback, this);

		//attitude estimation
		sprintf(msg_name,"/vehicle%d/yaw_est",m_group_index);
		m_subs.m_yawsub = nh.subscribe<crazyflie_driver::Yaw_est>(msg_name,5,&Swarm_Controller::att_estCallback, this);
		
		sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",m_group_index);
		m_subs.m_rawsub = nh.subscribe<easyfly::raw_ctrl_sp>(msg_name,5,&Swarm_Controller::rawctrlCallback, this);

		sprintf(msg_name,"/vehicle%d/tf_prefix",m_group_index);
		m_subs.cfIMUsub = nh.subscribe<sensor_msgs::Imu>(msg_name,5,&Swarm_Controller::cfIMUCallback, this);

		sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",m_group_index);
		m_subs.m_possub = nh.subscribe<easyfly::pos_ctrl_sp>(msg_name,5,&Swarm_Controller::posctrlCallback, this);

		sprintf(msg_name,"/vehicle%d/trj_ctrl_sp",m_group_index);
		m_subs.m_trjsub = nh.subscribe<easyfly::trj_ctrl_sp>(msg_name,5,&Swarm_Controller::trjctrlCallback, this);
		
		m_subs.m_cmdsub = nh.subscribe<easyfly::commands>("/commands",5,&Swarm_Controller::cmdCallback, this);
		
	}

	void run(double frequency)
	{
		ros::NodeHandle node;
		node.getParam("/flight_mode", m_flight_mode);
		
		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Swarm_Controller::iteration, this);
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
			m_pubs.m_outputpub.publish(m_output);
		}
		else{
			switch(m_flight_mode){
				case MODE_RAW:{

					m_output.att_sp.x = m_ctrl.raw.raw_att_sp.x;
					m_output.att_sp.y = m_ctrl.raw.raw_att_sp.y;
					m_output.att_sp.z = m_ctrl.raw.raw_att_sp.z;
					m_output.throttle = m_ctrl.raw.throttle;
					m_pubs.m_outputpub.publish(m_output);
				}//case MODE_RAW
				break;
				case MODE_POS:{

					if(m_cmd.flight_state!=Idle){
					m_sp_vecs.v_vel_sp(0) = m_pidX.pp_update((m_est_vecs.m_pos_est)(0), (m_sp_vecs.v_posctrl_posSp)(0));
					m_sp_vecs.v_vel_sp(1) = m_pidY.pp_update((m_est_vecs.m_pos_est)(1), (m_sp_vecs.v_posctrl_posSp)(1));
					m_sp_vecs.v_vel_sp(2) = m_pidZ.pp_update((m_est_vecs.m_pos_est)(2), (m_sp_vecs.v_posctrl_posSp)(2));

					m_sp_vecs.v_vel_sp(0) += (m_sp_vecs.v_posctrl_velFF)(0) * m_pidX.ff();
					m_sp_vecs.v_vel_sp(1) += (m_sp_vecs.v_posctrl_velFF)(1) * m_pidY.ff();
					m_sp_vecs.v_vel_sp(2) += (m_sp_vecs.v_posctrl_velFF)(2) * m_pidZ.ff();

					m_sp_vecs.v_acc_sp(0) = m_pidX.pid_update((m_est_vecs.m_vel_est)(0), (m_sp_vecs.v_vel_sp)(0));
					m_sp_vecs.v_acc_sp(1) = m_pidY.pid_update((m_est_vecs.m_vel_est)(1), (m_sp_vecs.v_vel_sp)(1));
					m_sp_vecs.v_acc_sp(2) = m_pidZ.pid_update((m_est_vecs.m_vel_est)(2), (m_sp_vecs.v_vel_sp)(2));

					m_sp_vecs.v_acc_sp(0) = m_pidX.pp_update((m_est_vecs.m_cfImuAcc)(0), (m_sp_vecs.v_acc_sp)(0));
					m_sp_vecs.v_acc_sp(1) = m_pidX.pp_update((m_est_vecs.m_cfImuAcc)(1), (m_sp_vecs.v_acc_sp)(1));
					m_sp_vecs.v_acc_sp(2) = m_pidX.pp_update((m_est_vecs.m_cfImuAcc)(2), (m_sp_vecs.v_acc_sp)(2));

					m_sp_vecs.v_acc_sp(2) += (float)GRAVITY / 1000.0;

					
					//float thrust_force = v_acc_sp.norm() * (float)VEHICLE_MASS / 1000.0;

					float thrust_force = m_sp_vecs.v_acc_sp.norm() * (float)VEHICLE_MASS / 2000.0;
					//printf("THRUST-FORCE: %f\n", thrust_force);
					//float thrust_force =2.0f;
					/*Vector3f body_z_sp = m_sp_vecs.v_acc_sp / m_sp_vecs.v_acc_sp.norm();
					Vector3f y_c;
					y_c(0) = -sin((m_sp_vecs.v_posctrl_attSp)(2));
					y_c(1) = cos((m_sp_vecs.v_posctrl_attSp)(2));
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
					m_output.att_sp.y = -asin(R_sp[2][0]);*/
					/*(m_sp_vecs.v_att_sp)(0) = atan2(R_sp[2][1], R_sp[2][2]);
					(m_sp_vecs.v_att_sp)(1) = -asin(R_sp[2][0]);*/

					//PRY control
					m_output.att_sp.x = m_pidRoll.pp_update((m_est_vecs.m_att_est)(0), (m_sp_vecs.v_posctrl_attSp)(0));
					m_output.att_sp.y = m_pidPitch.pp_update((m_est_vecs.m_att_est)(1), (m_sp_vecs.v_posctrl_attSp)(1));
					m_output.att_sp.z = m_pidYaw.pp_update((m_est_vecs.m_att_est)(2), (m_sp_vecs.v_posctrl_attSp)(2));
					
					/*m_output.att_sp.x = v_att_sp(0);
					m_output.att_sp.y = v_att_sp(1);
					m_output.att_sp.z = v_att_sp(2);*/
					m_output.throttle = thrust_force * 1000.0 / MAX_THRUST;
					//printf("OUTPUT:  %f, %f, %f, %f\n", m_output.att_sp.x,m_output.att_sp.y,m_output.att_sp.z,m_output.throttle);
					m_pubs.m_outputpub.publish(m_output);
				}//if flight_mode!=Idle
			}//case MODE_POS
				break;
				case MODE_TRJ:{
					
				}
				break;
				default:
				break;
			}//end switch flight mode
		}//end if cut
	}

	void cfIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		(m_est_vecs.m_cfImuAcc)(0)=msg->linear_acceleration.x;
		(m_est_vecs.m_cfImuAcc)(1)=msg->linear_acceleration.y;
		(m_est_vecs.m_cfImuAcc)(2)=msg->linear_acceleration.z;
		//TODO: low pass filter
		(m_est_vecs.m_cfImuVel)(0)=itergrate_update((m_est_vecs.m_cfImuAcc)(0));
		(m_est_vecs.m_cfImuVel)(1)=itergrate_update((m_est_vecs.m_cfImuAcc)(1));
		(m_est_vecs.m_cfImuVel)(2)=itergrate_update((m_est_vecs.m_cfImuAcc)(2));

		(m_est_vecs.m_vel_est)(0) = ((m_est_vecs.m_cfImuVel)(0)+(m_est_vecs.m_vel_est)(0))/2.0f;
		(m_est_vecs.m_vel_est)(1) = ((m_est_vecs.m_cfImuVel)(1)+(m_est_vecs.m_vel_est)(1))/2.0f;
		(m_est_vecs.m_vel_est)(2) = ((m_est_vecs.m_cfImuVel)(2)+(m_est_vecs.m_vel_est)(2))/2.0f;

		(m_est_vecs.m_cfImuPos)(0)=itergrate_update((m_est_vecs.m_cfImuVel)(0));
		(m_est_vecs.m_cfImuPos)(1)=itergrate_update((m_est_vecs.m_cfImuVel)(1));
		(m_est_vecs.m_cfImuPos)(2)=itergrate_update((m_est_vecs.m_cfImuVel)(2));

		(m_est_vecs.m_pos_est)(0) = ((m_est_vecs.m_pos_est)(0) + (m_est_vecs.m_cfImuPos)(0))/2.0f;
		(m_est_vecs.m_pos_est)(1) = ((m_est_vecs.m_pos_est)(1) + (m_est_vecs.m_cfImuPos)(1))/2.0f;
		(m_est_vecs.m_pos_est)(2) = ((m_est_vecs.m_pos_est)(2) + (m_est_vecs.m_cfImuPos)(2))/2.0f;

		}

	void vicon_Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
	{
		
		if(m_is_pos_original)
		{
			printf("HELLO, VICON!!!\n");
			(m_est_vecs.m_lpos)(0) = msg->transform.translation.x;
			(m_est_vecs.m_lpos)(1) = msg->transform.translation.y;
			(m_est_vecs.m_lpos)(2) = msg->transform.translation.z;
			ros::Time rightnow = ros::Time::now();
			m_times.m_lpos_time = rightnow;
			m_is_pos_original = false;
		}
		else
		{	easyfly::pos_est posmsg;
			//printf("m_is_pos_original: %d\n",m_is_pos_original );
			ros::Time rightnow = ros::Time::now();
			double dt = rightnow.toSec() - m_times.m_lpos_time.toSec();
			m_times.m_lpos_time = rightnow;
	
			(m_est_vecs.m_pos_est)(0) = msg->transform.translation.x;		
			(m_est_vecs.m_pos_est)(1) = msg->transform.translation.y;
			(m_est_vecs.m_pos_est)(2) = msg->transform.translation.z;

			posmsg.pos_est.x = (m_est_vecs.m_pos_est)(0);
			posmsg.pos_est.y = (m_est_vecs.m_pos_est)(1);
			posmsg.pos_est.z = (m_est_vecs.m_pos_est)(2);
			
			(m_est_vecs.m_vel_est)(0) = ((m_est_vecs.m_pos_est)(0) - (m_est_vecs.m_lpos)(0))/dt;
			(m_est_vecs.m_vel_est)(1) = ((m_est_vecs.m_pos_est)(1) - (m_est_vecs.m_lpos)(1))/dt;
			(m_est_vecs.m_vel_est)(2) = ((m_est_vecs.m_pos_est)(2) - (m_est_vecs.m_lpos)(2))/dt;
			posmsg.vel_est.x = (m_est_vecs.m_vel_est)(0);
			posmsg.vel_est.y = (m_est_vecs.m_vel_est)(1);
			posmsg.vel_est.z = (m_est_vecs.m_vel_est)(2);
	
			m_pubs.m_pos_estpub.publish(posmsg);
			(m_est_vecs.m_lpos)(0)=(m_est_vecs.m_pos_est)(0);
			(m_est_vecs.m_lpos)(1)=(m_est_vecs.m_pos_est)(1);
			(m_est_vecs.m_lpos)(2)=(m_est_vecs.m_pos_est)(2);
			writeData_bin("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/pos_est_Vicon.dat",m_est_vecs.m_pos_est);
			writeData_bin("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/vel_est_Vicon.dat",m_est_vecs.m_vel_est);
		}
	}

	void att_estCallback(const crazyflie_driver::Yaw_est::ConstPtr& est)
	{	
		(m_est_vecs.m_att_est)(0) = est->Roll_est;
		(m_est_vecs.m_att_est)(1) = est->Pitch_est;
		(m_est_vecs.m_att_est)(2) = est->Yaw_est;
		writeData_bin("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/att_est.dat",m_est_vecs.m_att_est);
		
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
		(m_sp_vecs.v_posctrl_posSp)(0) = ctrl->pos_sp.x;
		(m_sp_vecs.v_posctrl_posSp)(1) = ctrl->pos_sp.y;
		(m_sp_vecs.v_posctrl_posSp)(2) = ctrl->pos_sp.z;

		(m_sp_vecs.v_posctrl_velFF)(0) = ctrl->vel_ff.x;
		(m_sp_vecs.v_posctrl_velFF)(1) = ctrl->vel_ff.y;
		(m_sp_vecs.v_posctrl_velFF)(2) = ctrl->vel_ff.z;

		(m_sp_vecs.v_posctrl_attSp)(0) = ctrl->roll_sp;
		(m_sp_vecs.v_posctrl_attSp)(1) = ctrl->pitch_sp;
		(m_sp_vecs.v_posctrl_attSp)(2) = ctrl->yaw_sp;
		
		writeData_bin("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/pos_ctrl.dat",m_sp_vecs.v_posctrl_posSp);
		writeData_bin("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/velff_ctrl.dat",m_sp_vecs.v_posctrl_velFF);
		writeData_bin("/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/att_ctrl.dat",m_sp_vecs.v_posctrl_attSp);
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
	float itergrate_update(float control_goal)
	{
		float goal_integrated;
		ros::Time time = ros::Time::now();
		float dt = time.toSec() - m_times.m_previousTime.toSec();
		goal_integrated = control_goal*dt;
		return goal_integrated;
	}
	void writeData_bin(const char* fname, Vector3f vec)//, int num_Data)
	{	
		FILE* file_ptr;
		file_ptr = fopen(fname,"ab");
		if( file_ptr == NULL){
			file_ptr = fopen(fname,"wb");
		}
		char inputf[50];
		sprintf(inputf,"%f %f %f\n",vec(0),vec(1),vec(2));
		fputs (inputf,file_ptr);
		fclose (file_ptr);
	}

};
int main(int argc, char **argv)
{
	char node_name[50];
	ros::init(argc, argv, "Swarm_Controller");
	ros::NodeHandle n("~");
	n.getParam("/joy_num", g_joy_num);
	Swarm_Controller Swarm_Controller(n);
	Swarm_Controller.run(50);
	return 0;
}