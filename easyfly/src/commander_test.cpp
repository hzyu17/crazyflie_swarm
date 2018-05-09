#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Imu.h"
#include <easyfly/commands.h>
#include <easyfly/pos_ctrl_sp.h>
#include <easyfly/raw_ctrl_sp.h>
#include <easyfly/trj_ctrl_sp.h>
#include <easyfly/pos_est.h>	
#include <easyfly/att_est.h>
#include <easyfly/Recording.h>
#include <easyfly/output.h>
#include <easyfly/Learning.h>
#include <easyfly/vicon_markernum.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "commons.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
#include "type_methode.h"
//#include "gl_declair.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


int g_vehicle_num=4;	
int g_joy_num=1;
const int DimOfVarSpace = 2;
const float max_thrust = 0.5827*1.3;
float FREQUENCY = 50.0f;
using namespace Eigen;
using namespace std;
using namespace cv;
#define _USE_MATH_DEFINES //PI
/*************YE Xin gl*/
vector<float> x_init_pos;
vector<float> y_init_pos;
vector<float> x_marker_init;
std::vector<float> yaw_manuel;
vector<float> y_marker_init;
vector<int> index_sequence;
float amp_coeff;
bool flag = false;
void give_index(int index)
{
	index_sequence.push_back(index);
}
void clear_index()
{
	index_sequence.clear();
}
/*************YE Xin gl*/
class Commander
{
private:
	std::vector<ros::Publisher> m_rawpub_v, m_pospub_v, m_trjpub_v, m_pos_est_v, m_outputpub, m_vicon_markernumpub;
	std::vector<ros::Publisher> m_cmdpub_v;
    
	ros::Subscriber m_viconMarkersub;
	std::vector<ros::Subscriber> m_joysub_v, m_estsub_v, m_estyawsub_v,m_imusub_v,m_recordingsub,m_attdiffsub_v,m_veldiffsub_v,m_countTrainsub_v;
	std::vector<vicon_bridge::Marker> m_markers;
	//std::vector<MatrixXf> m_Connection_Matrix;
	MatrixXf m_Connection_Matrix;
	/******************
	landing params
	*******************/
	bool takeoff_switch;
	bool takeoff_condition;
	float takeoff_objective_height;
	float takeoff_safe_distance;
	float takeoff_low_rate;
	float takeoff_high_rate;
	float m_landspeed;
	float m_landsafe;  
	std::vector<float> m_thetaPos, m_radius_Pos, m_thetaPosS1, m_radius_PosS1;
	std::vector<float> yaw_bias;//walt
	bool yaw_manuel_ready;//walt

	std::vector<bool> yaw_est_ready;
	float _dt_deriv;
	int m_flight_mode;
	bool reset_takeoff,isFirstVicon;
	float m_takeoff_switch_Auto;
	float m_takeoff_switch_Hover;
	float m_land_switch_idle;
	bool isGotAtt, isGotPos, isFirstPos, isFirsrAtt, isFirstCircling, isHovering;
	float m_gamma;


	//For sequence initialization
	Mat src = Mat(Size(1000,1000), CV_8UC3, Scalar(0));

	//MODE_RAW 0
	//MODE_POS 1
	//MODE_TRJ 2
	int m_flight_state;
	int m_fl_state_after_hover;
	float m_hovering_time;
	float m_hover_duration;

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
	std::vector<Vector3f> swarm_pos;
	std::vector<Vector3f> m_hover_pos;
	std::vector<Vector4f> m_attoutput;

	ros::Time m_last_time_vicon;
	ros::Time m_this_time_vicon; 
	std::vector<Vector3f> m_swarm_pos;//the modifiable version of swarm_pos
	easyfly::commands m_cmd_msg;
	std::vector<Vector3f> _takeoff_Pos;
	Vector3f m_vel_ff, m_acc_sp,  m_last_rateSp;
	easyfly::pos_est m_pos_estmsg;
	easyfly::output m_output;

/***************
learning params
****************/
	float m_gamma_theta;
	float m_gamma_vel;
	int m_count_state;
	int m_count_training;
	float m_T1, m_T2, m_T3, m_lenS1, m_vel_desNorm;
	bool isFirstS1,isFirstS2,isFirstS3,isNewTrain;
	std::vector<Vector3f> swarm_vel_err, swarm_att_err;
	float z_ground;

public:
	Commander(ros::NodeHandle& nh)
	:m_rawpub_v(g_vehicle_num)
	,m_imusub_v(g_vehicle_num)
	,m_attdiffsub_v(g_vehicle_num)
	,m_veldiffsub_v(g_vehicle_num)
	,m_pospub_v(g_vehicle_num)
	,m_trjpub_v(g_vehicle_num)
	,m_vicon_markernumpub(g_vehicle_num)
	,m_joysub_v(g_joy_num)
	,m_estsub_v(g_vehicle_num)
	,m_estyawsub_v(g_vehicle_num)
	,m_recordingsub(g_vehicle_num)
	,m_countTrainsub_v(g_vehicle_num)
	,m_joy_v(g_joy_num)
	,m_ctrl_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	,m_att_est_v(g_vehicle_num)
	,m_pos_est_v(g_vehicle_num)
	,yaw_bias(g_vehicle_num)//walt
	,m_attoutput(g_vehicle_num)
	,swarm_pos(g_vehicle_num)
	,m_hover_pos(g_vehicle_num)
	,swarm_vel_err(g_vehicle_num)
	,swarm_att_err(g_vehicle_num)
	,m_cmd_msg()
	,takeoff_switch(false)
	,yaw_manuel_ready(false)//walt
	,yaw_est_ready(g_vehicle_num)//walt
	,takeoff_condition(true)
	,isGotAtt(false)
	,isFirstVicon(true)
	,isGotPos(false)
	,isFirstPos(true)
	,isFirsrAtt(true)
	,isFirstCircling(true)
	,isHovering(false)
	,takeoff_objective_height(1.2f)
	,takeoff_safe_distance(0.0f)
	,takeoff_low_rate(0.2f)
	,takeoff_high_rate(0.3f)
	,reset_takeoff(false)
	,m_landspeed(0.2f)
	,m_thetaPos(g_vehicle_num)
	,m_thetaPosS1(g_vehicle_num)
	,m_radius_Pos(g_vehicle_num)
	,m_radius_PosS1(g_vehicle_num)
    ,_takeoff_Pos(g_vehicle_num)
    ,m_cmdpub_v(g_vehicle_num)
	,m_land_switch_idle(0.1f)
	,m_takeoff_switch_Auto(0.2f)
	,m_takeoff_switch_Hover(0.2f)
	,m_hovering_time(0)
	,m_hover_duration(-1)
	,m_Connection_Matrix(g_vehicle_num,g_vehicle_num)
	,m_gamma(0.5)
	,m_gamma_theta(0.5)
	,m_gamma_vel(0.5)
	,m_vel_desNorm(3.0f)
	{
		//printf("entering commander!!");
		m_vel_ff.setZero();
		m_acc_sp.setZero();
		//_takeoff_Pos.setZero();
		m_last_rateSp.setZero();
		m_last_time_vicon = ros::Time::now();
		m_this_time_vicon = ros::Time::now();
		m_flight_state = Idle;
        
		m_fl_state_after_hover = Hovering;

		char msg_name[50];
		m_cmd_msg.cut = 0;
		m_cmd_msg.flight_state = Idle;
		
		
		for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",i);
			m_pospub_v[i] = nh.advertise<easyfly::pos_ctrl_sp>(msg_name, 1);

            sprintf(msg_name,"/vehicle%d/commands",i);
            m_cmdpub_v[i] = nh.advertise<easyfly::commands>(msg_name,1);

			sprintf(msg_name,"/vehicle%d/pos_est", i); 
			m_pos_est_v[i] = nh.advertise<easyfly::pos_est>(msg_name, 1000);
			
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
		_dt_deriv = e.current_real.toSec() - e.last_real.toSec();
		time_elapse += _dt_deriv;
        
		//cut off
		if(m_joy_v[0].curr_buttons[4] == 1 && m_joy_v[0].curr_buttons[5] == 1){
			m_cmd_msg.cut = 1;
		}
		else{
			switch(m_flight_mode){
				case MODE_POS:{
					if(m_joy_v[0].changed_arrow[1] == false){ 

						//faire rien..
					}

					else if(m_joy_v[0].changed_arrow[1] == true && m_joy_v[0].curr_arrow[1] == 1){//take off
						m_joy_v[0].changed_arrow[1] = false;
						if(m_flight_state == Idle){
							m_flight_state = TakingOff;
                            m_cmd_msg.flight_state = m_flight_state;
                            for(int v=0;v<g_vehicle_num;++v)
                            {
                                m_cmdpub_v[v].publish(m_cmd_msg);
                                ros::Duration(3.0).sleep(); 
                            }
                            m_flight_state = Idle;
							//printf("hello!!!!!run\n");
						}
                    }
				}//end case posctrl mode
				break;
				default:
				break;
			} //end switch mode
		}//end of cut off case		
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
	
	void command_takeoff(int i, float dt)
	{
        
		reset_takeoff = false;
		//g_statusFlight = statusTakingoff;		
		m_ctrl_v[i].posctrl_msg.pos_sp.x = _takeoff_Pos[i](0);
		m_ctrl_v[i].posctrl_msg.pos_sp.y = _takeoff_Pos[i](1); 
      
		if(m_ctrl_v[i].posctrl_msg.pos_sp.z >= _takeoff_Pos[i](2) + takeoff_objective_height)
		{
			takeoff_switch = false;
			m_ctrl_v[i].posctrl_msg.pos_sp.z = _takeoff_Pos[i](2) + takeoff_objective_height;
		}
		else
		{
	    	if(takeoff_condition) //in case of a sudden thrust at the beggining..
	    	{
	    		m_ctrl_v[i].posctrl_msg.pos_sp.z = _takeoff_Pos[i](2) - takeoff_safe_distance;
	    		takeoff_condition = false;
	    	}
	    	else
	    	{
	    		if(m_ctrl_v[i].posctrl_msg.pos_sp.z < _takeoff_Pos[i](2) + 0.2f)
	    		{
	    			m_ctrl_v[i].posctrl_msg.pos_sp.z += takeoff_low_rate * dt;

	    			m_ctrl_v[i].posctrl_msg.vel_ff.z = takeoff_low_rate;

	    			m_ctrl_v[i].posctrl_msg.acc_sp.z = deriv_f(takeoff_low_rate, m_last_rateSp(2), dt);
	    			m_last_rateSp(2) = takeoff_low_rate;
	    		}
	    		else
	    		{
	    			//printf("hello,high speed!\n");
	    			m_ctrl_v[i].posctrl_msg.pos_sp.z += takeoff_high_rate * dt;

	    			m_ctrl_v[i].posctrl_msg.vel_ff.z = takeoff_high_rate;

	    			m_ctrl_v[i].posctrl_msg.acc_sp.z = deriv_f(takeoff_high_rate, m_last_rateSp(2), dt);
	    			m_last_rateSp(2) = takeoff_high_rate;
	    		}		
	      	}
	    }
	}
/*************************
	Cmd Circling
*************************/
	void cal_radius(int index)
	{
		m_radius_Pos[index] = sqrt(swarm_pos[index](1)*swarm_pos[index](1) + swarm_pos[index](0)*swarm_pos[index](0));
	}

	void cal_theta(int index)
	{
		m_thetaPos[index] = atan2(swarm_pos[index](1),swarm_pos[index](0));
	}

	void cmd_to_radius(int index, float object_radius, float circle_err, float outWards_step, float z_object)
	{
		if(m_radius_Pos[index]>object_radius)
		{	
			m_ctrl_v[index].posctrl_msg.pos_sp.x -= outWards_step*cos(m_thetaPos[index]);
			m_ctrl_v[index].posctrl_msg.pos_sp.y -= outWards_step*sin(m_thetaPos[index]);
			m_ctrl_v[index].posctrl_msg.pos_sp.z = z_object;	

		}
		else if(m_radius_Pos[index]<object_radius)
		{
			m_ctrl_v[index].posctrl_msg.pos_sp.x += outWards_step*cos(m_thetaPos[index]);
			m_ctrl_v[index].posctrl_msg.pos_sp.y += outWards_step*sin(m_thetaPos[index]);
			m_ctrl_v[index].posctrl_msg.pos_sp.z = z_object;	
		}
	}
	
/******************
	Cmd landing
*******************/
	void control_landing(int i, float dt)
	{
		//printf("########### Vehicle %d is landing!! #######\n",i);
		reset_takeoff = false;
		m_ctrl_v[i].posctrl_msg.pos_sp.x = swarm_pos[i](0);
		m_ctrl_v[i].posctrl_msg.pos_sp.y = swarm_pos[i](1);
		m_ctrl_v[i].posctrl_msg.pos_sp.z -= m_landspeed * dt;
		if (swarm_pos[i](2)<=_takeoff_Pos[i](2) + m_land_switch_idle)
		{
			m_flight_state = Idle;
		}

	}


    

};

int main(int argc, char **argv)
{
	
//  int ret = init_scan(argc, argv);
	//glutInit(&argc, argv);
	ros::init(argc, argv, "commander_test");
	ros::NodeHandle n;
	// ros::NodeHandle n;
	n.getParam("/g_vehicle_num", g_vehicle_num);
	n.getParam("/joy_num", g_joy_num);

	Commander commander(n);	
	commander.run(100);
  return 0;


}

