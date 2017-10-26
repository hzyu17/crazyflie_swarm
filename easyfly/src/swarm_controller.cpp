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
#include "crazyflie_driver/Att_est.h"
//#include "type_methode.h"
#include "control.h"
#include <fstream>
using namespace Eigen;
using namespace std;
#define _USE_MATH_DEFINES //PI

int g_joy_num=1;
int num_record = 5000;
int loop_record = 0;
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

class Swarm_Controller: private Controller
{
private:
	int m_group_index;
	int m_vehicle_num;
	//PID m_pidX,m_pidY,m_pidZ,m_pidYaw;
	/*PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	PID m_pidRoll;
	PID m_pidPitch;
	PID m_pidYaw;*/
	float m_pidx_kp;
	int m_flight_state, m_flight_mode;
	bool isFirstVicon;
	bool isFirstposSp;
	bool isFirstAccIMU;
	char m_resFnameRoot[150];
	struct M_Ctrl
	{
		//easyfly::pos_ctrl_sp pos;
		easyfly::raw_ctrl_sp raw;
		easyfly::trj_ctrl_sp trj;
	};
	M_Ctrl m_ctrl;

	struct M_Pubs{
		ros::Publisher m_outputpub, m_pos_estpub;
	};
	M_Pubs m_pubs;
	struct M_subs{
		ros::Subscriber m_pos_estpub, m_att_estsub, m_accsub, m_viconsub, m_yawsub;
		ros::Subscriber m_rawsub, m_possub, m_trjsub, cfIMUsub;
		ros::Subscriber m_cmdsub;
	};
	M_subs m_subs;
	M_est_Vecs m_est_vecs;
	M_sp_Vecs m_sp_vecs;
	struct M_times
	{
		ros::Time m_previousTime, m_latt_time, m_lposEst_time, m_lposSp_time, m_IntePrevious; 
	};
	M_times m_times;

	Vector4f v_posctrl_posSp, v_posctrl_output;
	easyfly::commands m_cmd;
	easyfly::output m_output;
	std::string tf_prefix;
	Att m_att;

public:
	
	Swarm_Controller(
		const ros::NodeHandle& n)
		: isFirstVicon(true)
		, isFirstAccIMU(true)
		, isFirstposSp(true)
		, m_cmd()
		, m_ctrl()
		, m_sp_vecs()
		, m_est_vecs()
		, m_pubs()
		, m_subs()
		, m_resFnameRoot()
		, m_att()
		
	{
		m_times.m_IntePrevious = ros::Time::now();
		m_times.m_previousTime = ros::Time::now();
		m_times.m_latt_time = ros::Time::now();
		m_times.m_lposSp_time = ros::Time::now();
		m_times.m_lposEst_time = ros::Time::now();  
		ros::NodeHandle nh("~");//~ means private param
		nh.getParam("group_index", m_group_index);

		sprintf(m_resFnameRoot,"/home/lucasyu/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/vehicle%d/",m_group_index);

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
		sprintf(msg_name,"/vehicle%d/att_est",m_group_index);
		m_subs.m_yawsub = nh.subscribe<crazyflie_driver::Att_est>(msg_name,5,&Swarm_Controller::att_estCallback, this);
		
		sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",m_group_index);
		m_subs.m_rawsub = nh.subscribe<easyfly::raw_ctrl_sp>(msg_name,5,&Swarm_Controller::rawctrlCallback, this);

		sprintf(msg_name,"/vehicle%d/tf_prefix/Imu",m_group_index);
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
						position_control(&m_est_vecs.m_pos_est, &m_sp_vecs.v_posctrl_posSp, &v_posctrl_output, &m_est_vecs.m_cfImuAcc, &m_att.R, dt);
						attitude_control(&m_sp_vecs.v_posctrl_posSp, &v_posctrl_output, &m_att.R, &m_est_vecs.m_gyro_est, dt);
						m_output.att_sp.x = v_posctrl_output(0);
						m_output.att_sp.y = v_posctrl_output(1);
						m_output.att_sp.z = v_posctrl_output(2);
						m_output.throttle = v_posctrl_output(3);
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
		//write files:
		if(loop_record<=num_record)
		{
			loop_record++;
			writeData_bin("pos_sp.dat",m_sp_vecs.v_posctrl_posSp);
			writeData_bin("pos_est_Vicon.dat",m_est_vecs.m_pos_est);
			//writeData_bin("ImuPos_est.dat",m_est_vecs.m_cfImuPos);
			
			/*writeData_bin("vel_sp.dat",m_sp_vecs.v_vel_sp);
			writeData_bin("vel_est_Vicon.dat",m_est_vecs.m_vel_est);
			writeData_bin("ImuVel_est.dat",m_est_vecs.m_cfImuVel);*/
			
			//writeData_bin("acc_sp.dat",m_sp_vecs.v_acc_sp);
			//writeData_bin("acc_est_Vicon.dat",m_est_vecs.m_cfImuAcc);
			writeData_bin("ImuAcc_est.dat",m_est_vecs.m_cfImuAcc);
	
			//writeData_bin("att_ctrl.dat",m_sp_vecs.v_att_sp);
			writeData_bin("att_est.dat",m_est_vecs.m_att_est);
		}

	}//iteration

	void cfIMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		(m_est_vecs.m_cfImuAcc)(0) = msg->linear_acceleration.x;
		(m_est_vecs.m_cfImuAcc)(1) = msg->linear_acceleration.y;
		(m_est_vecs.m_cfImuAcc)(2) = msg->linear_acceleration.z;
		(m_est_vecs.m_gyro_est)(0) = msg->angular_velocity.x;
		(m_est_vecs.m_gyro_est)(1) = msg->angular_velocity.y;
		(m_est_vecs.m_gyro_est)(2) = msg->angular_velocity.z;
		if(isFirstAccIMU){
			resetaccController(&m_est_vecs.m_cfImuAcc);
			isFirstAccIMU = false;
		}
	}

	void vicon_Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
	{
		printf("HELLO, VICON!!!\n");
		(m_est_vecs.m_pos_est)(0) = msg->transform.translation.x;
		(m_est_vecs.m_pos_est)(1) = msg->transform.translation.y;
		(m_est_vecs.m_pos_est)(2) = msg->transform.translation.z;
		if(isFirstVicon)
		{
			resetposController(&m_est_vecs.m_pos_est);
			isFirstVicon = false;
		}
		
	}

	void att_estCallback(const crazyflie_driver::Att_est::ConstPtr& est)
	{	
		(m_est_vecs.m_att_est)(0) = est->att_est.x;
		(m_est_vecs.m_att_est)(1) = est->att_est.y;
		(m_est_vecs.m_att_est)(2) = est->att_est.z;
		euler2rotation(&m_est_vecs.m_att_est, &m_att.R);
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
	
		if(isFirstposSp)
		{
			resetposSp(&m_sp_vecs.v_posctrl_posSp);
			isFirstposSp = false;	
		}
		
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
	
	void writeData_bin(const char* fname_input, Vector3f vec)//, int num_Data)
	{	
		//rename
		char fname[250];
		sprintf(fname,"%s%s",m_resFnameRoot,fname_input);
		//create file
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