#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/MagneticField.h>
//#include <sensor_msgs/>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <easyfly/commands.h>
#include <easyfly/pos_ctrl_sp.h>
#include <easyfly/raw_ctrl_sp.h>
#include <easyfly/trj_ctrl_sp.h>
#include <easyfly/pos_est.h>	
#include <easyfly/att_est.h>
#include <crazyflie_driver/Att_est.h>
#include "commons.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
int g_vehicle_num=1;
int g_joy_num=1;
using namespace Eigen;

class Commander
{
private:
	std::vector<ros::Publisher> m_rawpub_v, m_pospub_v, m_trjpub_v;
	ros::Publisher m_cmdpub;
	std::vector<ros::Subscriber> m_joysub_v, m_estsub_v, m_estyawsub_v;
	float takeoff_height;
	bool takeoff_switch;
	bool takeoff_condition;
	float takeoff_objective_height;
	float takeoff_safe_distance;
	float takeoff_low_rate;
	float takeoff_high_rate;
	float m_landspeed;
	float m_landsafe;
	int m_flight_mode;
	bool reset_takeoff;
	float m_takeoff_switch_Auto;
	float m_takeoff_switch_Hover;
	float m_land_switch_idle;

	//MODE_RAW 0
	//MODE_POS 1
	//MODE_TRJ 2
	int m_flight_state;
//	float m_velff_xy_P, m_velff_z_P;
	struct M_Joy
	{
		bool curr_buttons[14];
		bool changed_buttons[14];
		float axes[4];
		int curr_arrow[2];
		bool changed_arrow[2];
	};
	std::vector<M_Joy> m_joy_v;
	struct M_Ctrl
	{
		easyfly::pos_ctrl_sp posctrl_msg;
		easyfly::raw_ctrl_sp rawctrl_msg;
		easyfly::trj_ctrl_sp trjctrl_msg;
	};
	std::vector<M_Ctrl> m_ctrl_v;
	std::vector<easyfly::pos_est> m_est_v;
	std::vector<Vector3f> m_att_est_v;
	easyfly::commands m_cmd_msg;

public:
	Commander(ros::NodeHandle& nh)
	:m_rawpub_v(g_vehicle_num)
	,m_pospub_v(g_vehicle_num)
	,m_trjpub_v(g_vehicle_num)
	,m_joysub_v(g_joy_num)
	,m_estsub_v(g_vehicle_num)
	,m_estyawsub_v(g_vehicle_num)
	,m_joy_v(g_joy_num)
	,m_ctrl_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	,m_att_est_v(g_vehicle_num)
	,m_cmd_msg()
	,takeoff_height(0.46f)
	,takeoff_switch(false)
	,takeoff_condition(true)
	,takeoff_objective_height(1.0f)
	,takeoff_safe_distance(0.1f)
	,takeoff_low_rate(0.2f)
	,takeoff_high_rate(0.3f)
	,reset_takeoff(false)
	,m_landspeed(0.2f)
	,m_land_switch_idle(0.1f)
	,m_takeoff_switch_Auto(0.4f)
	,m_takeoff_switch_Hover(0.4f)
	{
		m_flight_state = Idle;
//		m_velff_xy_P = 0.6;
//		m_velff_z_P = 0.5;
		char msg_name[50];
		m_cmd_msg.cut = 0;
		m_cmd_msg.flight_state = Idle;
		m_cmd_msg.l_flight_state = Idle;
		m_cmdpub = nh.advertise<easyfly::commands>("/commands",1);
		
		for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",i);
			m_rawpub_v[i] = nh.advertise<easyfly::raw_ctrl_sp>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",i);
			m_pospub_v[i] = nh.advertise<easyfly::pos_ctrl_sp>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/trj_ctrl_sp",i);
			m_trjpub_v[i] = nh.advertise<easyfly::trj_ctrl_sp>(msg_name, 1);
			sprintf(msg_name,"/vehicle%d/att_est",i);
			m_estyawsub_v[i] = nh.subscribe<crazyflie_driver::Att_est>(msg_name,5,boost::bind(&Commander::att_estCallback, this, _1, i));
			sprintf(msg_name,"/vehicle%d/pos_est",i);
			m_estsub_v[i] = nh.subscribe<easyfly::pos_est>(msg_name,5,boost::bind(&Commander::pos_estCallback, this, _1, i));
		}
		for(int i=0;i<g_joy_num;i++){
			sprintf(msg_name,"/joygroup%d/joy",i);
			m_joysub_v[i] = nh.subscribe<sensor_msgs::Joy>(msg_name,5,boost::bind(&Commander::joyCallback, this, _1, i));
		}
		
	}
	void run(double frequency)
	{
		ros::NodeHandle node;
		node.getParam("flight_mode", m_flight_mode);

		char msg_name[50];
		/*for(int i=0;i<g_vehicle_num;i++){
			
		}*/
		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Commander::iteration, this);
		ros::spin();
	}
	void iteration(const ros::TimerEvent& e)
	{	

		static float time_elapse = 0;
		float dt = e.current_real.toSec() - e.last_real.toSec();
		time_elapse += dt;

		//cut off
		if(m_joy_v[0].curr_buttons[4] == 1 && m_joy_v[0].curr_buttons[5] == 1){
			m_cmd_msg.cut = 1;
		}
		else{	
			switch(m_flight_mode){
				case MODE_RAW:{
					for(int i=0; i<g_vehicle_num;i++){
						m_ctrl_v[i].rawctrl_msg.raw_att_sp.x = -m_joy_v[0].axes[0] * 30 * DEG2RAD;//+-1
						m_ctrl_v[i].rawctrl_msg.raw_att_sp.y = m_joy_v[0].axes[1] * 30 * DEG2RAD;
						m_ctrl_v[i].rawctrl_msg.raw_att_sp.z = m_joy_v[0].axes[2] * 20 * DEG2RAD;//rate
						m_ctrl_v[i].rawctrl_msg.throttle = m_joy_v[0].axes[3];//0-1
						if(m_ctrl_v[i].rawctrl_msg.throttle<0){
							m_ctrl_v[i].rawctrl_msg.throttle=0;
						}
						m_rawpub_v[i].publish(m_ctrl_v[i].rawctrl_msg);
					}
				}
				break;
				case MODE_POS:{

					if(m_joy_v[0].changed_arrow[1] == false){ 
						//faire rien..
					}

					else if(m_joy_v[0].changed_arrow[1] == true && m_joy_v[0].curr_arrow[1] == 1){//take off
						m_joy_v[0].changed_arrow[1] = false;
						if(m_flight_state == Idle){
							for(int i=0;i<g_vehicle_num;i++){
								posspReset(i);
								yawspReset(i);
							}
							m_flight_state = TakingOff;
						}
					}
					else if(m_joy_v[0].changed_arrow[1] == true && m_joy_v[0].curr_arrow[1] == -1){//land
						m_joy_v[0].changed_arrow[1] = false;
						if(m_flight_state == Automatic || m_flight_state == TakingOff)
							m_flight_state = Landing;
					}
					

					switch(m_flight_state){
						case Idle:{
							for(int i=0;i<g_vehicle_num;i++){
							}
							//all motors off
						}
						break;
						case Automatic:{
							for(int i=0;i<g_vehicle_num;i++){
								float pos_move_rate[3];
								pos_move_rate[0] = m_joy_v[i].axes[0] * 2.0;
								pos_move_rate[1] = m_joy_v[i].axes[1] * 2.0;
								pos_move_rate[2] = m_joy_v[i].axes[3] * 1.0;
								m_ctrl_v[i].posctrl_msg.pos_sp.x += pos_move_rate[0];
								m_ctrl_v[i].posctrl_msg.pos_sp.y += pos_move_rate[1];
								m_ctrl_v[i].posctrl_msg.pos_sp.z += pos_move_rate[2];
								m_ctrl_v[i].posctrl_msg.vel_ff.x = pos_move_rate[0];
								m_ctrl_v[i].posctrl_msg.vel_ff.y = pos_move_rate[1];
								m_ctrl_v[i].posctrl_msg.vel_ff.z = pos_move_rate[2];
								float yaw_move_rate = m_joy_v[i].axes[2] * 20 * DEG2RAD;
								m_ctrl_v[i].posctrl_msg.yaw_sp += yaw_move_rate;
								m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
							}
						}
						break;
						case TakingOff:{
							for(int i=0;i<g_vehicle_num;i++){
								command_takeoff(i,dt);
								m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
							}
							//TODO judge if it is time to get into Automatic
						}
						break;
						case Landing:{
							for(int i=0;i<g_vehicle_num;i++){
								control_landing(i, dt);
							}
							//TODO judge if it is time to get into Idle
						}
						break;
						case Hovering:{
							for(int i=0;i<g_vehicle_num;i++){
								posspReset(i);
								yawspReset(i);
								m_ctrl_v[i].posctrl_msg.pos_sp.z = 1.0f;
								m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
							}
						}
						default:
						break;
					}//end switch state
				}//end case posctrl mode
				break;
				case MODE_TRJ:{

				}
				break;
				default:
				break;
			}//end switch mode
		}//end of cut off case
		m_cmd_msg.flight_state = m_flight_state;
		m_cmdpub.publish(m_cmd_msg);
		m_cmd_msg.l_flight_state = m_flight_state;
	}

	void posspReset(int index)
	{
		m_ctrl_v[index].posctrl_msg.pos_sp.x = m_est_v[index].pos_est.x;
		m_ctrl_v[index].posctrl_msg.pos_sp.y = m_est_v[index].pos_est.y;
		m_ctrl_v[index].posctrl_msg.pos_sp.z = m_est_v[index].pos_est.z;
		m_ctrl_v[index].posctrl_msg.vel_ff.x = 0;
		m_ctrl_v[index].posctrl_msg.vel_ff.y = 0;
		m_ctrl_v[index].posctrl_msg.vel_ff.z = 0;
	}
	void yawspReset(int index)
	{
		m_ctrl_v[index].posctrl_msg.roll_sp = (m_att_est_v[index])(0);
		m_ctrl_v[index].posctrl_msg.pitch_sp = (m_att_est_v[index])(1);
		m_ctrl_v[index].posctrl_msg.yaw_sp = (m_att_est_v[index])(2);
		//m_ctrl_v[index].posctrl_msg.pitch_sp = m_att_est_v[index].Pitch_est;
		//m_ctrl_v[index].posctrl_msg.yaw_sp = m_att_est_v[index].Yaw_est;

	}
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index)
	{
		//0 PosCtrl, 1 AttCtrl
		joy_index=0;
		#define MAX_JOYS 5
		static bool l_buttons[MAX_JOYS][14];//at most 5 joysticks
		static int l_arrow[MAX_JOYS][2];
		for(int i=0;i<14;i++){
			m_joy_v[joy_index].curr_buttons[i] = joy->buttons[i];
			if(m_joy_v[joy_index].curr_buttons[i] != l_buttons[joy_index][i])
				m_joy_v[joy_index].changed_buttons[i] = true;
			else
				;//changed_buttons cleared in iteration
		}
		for(int i=0;i<14;i++){
			l_buttons[joy_index][i] = m_joy_v[joy_index].curr_buttons[i];
		}
										
		m_joy_v[joy_index].axes[0] = joy->axes[3];//roll
		m_joy_v[joy_index].axes[1] = -joy->axes[4];//pitch
		m_joy_v[joy_index].axes[2] = -joy->axes[0];//yaw
		m_joy_v[joy_index].axes[3] = joy->axes[1];//thr
		if(joy->axes[6]>0.5)//left and right button belong to one axes
			m_joy_v[joy_index].curr_arrow[0] = 1;
		else if(joy->axes[6]<-0.5)
			m_joy_v[joy_index].curr_arrow[0] = -1;
		else
			m_joy_v[joy_index].curr_arrow[0] = 0;
		if(joy->axes[7]>0.5)//up and down button is belong to one axes
			m_joy_v[joy_index].curr_arrow[1] = 1;
		else if(joy->axes[7]<-0.5)
			m_joy_v[joy_index].curr_arrow[1] = -1;
		else
			m_joy_v[joy_index].curr_arrow[1] = 0;
		for(int i=0;i<2;i++){
			if(m_joy_v[joy_index].curr_arrow[i] != l_arrow[joy_index][i])
				m_joy_v[joy_index].changed_arrow[i]=true;
		}
		for(int i=0;i<2;i++){
			l_arrow[joy_index][i] = m_joy_v[joy_index].curr_arrow[i];
		}
	}
	void pos_estCallback(const easyfly::pos_est::ConstPtr& est, int vehicle_index)
	{
		//printf("POS_EST:  %f, %f, %f\n", m_est_v[vehicle_index].pos_est.x,m_est_v[vehicle_index].pos_est.y,m_est_v[vehicle_index].pos_est.z);
		m_est_v[vehicle_index].pos_est.x = est->pos_est.x;
		m_est_v[vehicle_index].pos_est.y = est->pos_est.y;
		m_est_v[vehicle_index].pos_est.z = est->pos_est.z;
		m_est_v[vehicle_index].vel_est.x = est->vel_est.x;
		m_est_v[vehicle_index].vel_est.y = est->vel_est.y;
		m_est_v[vehicle_index].vel_est.z = est->vel_est.z;
	}
	void att_estCallback(const crazyflie_driver::Att_est::ConstPtr& est, int vehicle_index)
	{
		//m_att_est_v[vehicle_index].Roll_est = est->Roll_est;
		(m_att_est_v[vehicle_index])(0) = est->att_est.x;
		(m_att_est_v[vehicle_index])(1) = est->att_est.y;
		(m_att_est_v[vehicle_index])(2) = est->att_est.z;

	}
	void command_takeoff(int i, float dt)
	{
		reset_takeoff = false;
//		g_statusFlight = statusTakingoff;		
		m_ctrl_v[i].posctrl_msg.pos_sp.x = m_est_v[i].pos_est.x;
		m_ctrl_v[i].posctrl_msg.pos_sp.y = m_est_v[i].pos_est.y;
		yawspReset(i);
		/*m_ctrl_v[i].posctrl_msg.yaw_sp = m_att_est_v[i].Yaw_est;
		m_ctrl_v[i].posctrl_msg.roll_sp = 0.0f;
		m_ctrl_v[i].posctrl_msg.pitch_sp = 0.0f;*/

		if(m_ctrl_v[i].posctrl_msg.pos_sp.z >= takeoff_height + takeoff_objective_height)
		{
			takeoff_switch = false;
			m_ctrl_v[i].posctrl_msg.pos_sp.z = takeoff_height + takeoff_objective_height;
		}
		else
		{
	    	if(takeoff_condition) //in case of a sudden thrust at the beggining..
	    	{
	    		//takeoff_height = m_est_v[i].pos_est.z; //reset the takingoff height
	    		m_ctrl_v[i].posctrl_msg.pos_sp.z = takeoff_height - takeoff_safe_distance;
	    		takeoff_condition = false;
	    	}
	    	else
	    	{
	    		if(m_ctrl_v[i].posctrl_msg.pos_sp.z < takeoff_height + takeoff_safe_distance)
	    		{
	    			m_ctrl_v[i].posctrl_msg.pos_sp.z += takeoff_low_rate * dt;
	    		}else
	    		{
	    			m_ctrl_v[i].posctrl_msg.pos_sp.z += takeoff_high_rate * dt;
	    		}
	    		/*if(m_est_v[i].pos_est.z < takeoff_objective_height + m_takeoff_switch_Auto)
	    		{
	    			m_flight_state = Automatic;
	    		}*/
	    		if(m_est_v[i].pos_est.z > takeoff_objective_height + m_takeoff_switch_Hover)
	    		{
	    			m_flight_state = Hovering;
	    		}
	      	}
	    }
	}

	void control_landing(int i, float dt)
	{
		reset_takeoff = false;
		m_ctrl_v[i].posctrl_msg.pos_sp.x = m_est_v[i].pos_est.x;
		m_ctrl_v[i].posctrl_msg.pos_sp.y = m_est_v[i].pos_est.y;
		m_ctrl_v[i].posctrl_msg.pos_sp.z -= m_landspeed * dt;
		if (m_est_v[i].pos_est.z<=takeoff_height + m_land_switch_idle)
		{
			m_flight_state = Idle;
		}
	}
};

int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
	ros::init(argc, argv, "commander");
	ros::NodeHandle n;
	// ros::NodeHandle n;
	n.getParam("/g_vehicle_num", g_vehicle_num);
	n.getParam("/joy_num", g_joy_num);
//	n.getParam("/flight_mode", g_flight_mode);this has moved to function run
	Commander commander(n);
	commander.run(50);


  return 0;


}
