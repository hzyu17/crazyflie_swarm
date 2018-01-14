#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <easyfly/commands.h>
#include <easyfly/state_est.h>
#include <easyfly/output.h>
#include <crazyflie_driver/num_vehiclepub.h>
#include "crazyflie_driver/Yaw_est.h"

#include <geometry_msgs/TransformStamped.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "commons.h"

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/Crazyflie.h>
int g_vehicle_num; //get from the num_vehicle msg
class Linker
{
private:
	std::vector<ros::Publisher> m_estpub_v;
	std::vector<ros::Subscriber> m_viconsub_v,m_yawsub_v;
	std::vector<easyfly::state_est> m_est_v;
	std::vector<easyfly::output> m_output_v;
	std::vector<geometry_msgs::Vector3> m_lpos_v;
	std::vector<ros::Time> m_lpos_time_v;
	std::vector<std::string> m_defaultUri_v, m_uri_v;
	std::vector<Crazyflie> m_cf_v;
	//vicon estimator:
	float m_vicon_freq;
	float m_vicon_pos_err;
	int m_index; //get from the yaw_msg


public:
	Linker(ros::NodeHandle& nh)
	:m_estpub_v(g_vehicle_num)
	,m_est_v(g_vehicle_num)
	,m_viconsub_v(g_vehicle_num)
	,m_lpos_v(g_vehicle_num)
	,m_lpos_time_v(g_vehicle_num)
	,m_defaultUri_v(g_vehicle_num)
	,m_uri_v(g_vehicle_num)
	,m_output_v(g_vehicle_num)
	,m_vicon_pos_err(0.1f)
	{
		char msg_name[50];
		for(int i=0;i<g_vehicle_num;i++){
			sprintf(msg_name,"/vicon/crazyflie%d/whole",i);
			m_viconsub_v[i] = nh.subscribe<geometry_msgs::TransformStamped>(msg_name,5,boost::bind(&Linker::viconCallback, this));
			sprintf(msg_name,"/vehicle%d/yaw_est",i);
			m_yawsub_v[i] = nh.subscribe<crazyflie_driver::Yaw_est>(msg_name,5,&Linker::yaw_estCallback, this);
			m_lpos_time_v[i] = ros::Time::now();

		}
	}
	void yaw_estCallback(const crazyflie_driver::Yaw_est::ConstPtr& msg)
	{	
		m_index = msg->group_index;
		m_est_v[m_index].pitch_est = msg->Pitch_est;
		m_est_v[m_index].yaw_est = msg->Yaw_est;
		m_est_v[m_index].roll_est = msg->Roll_est;
		m_estpub_v[m_index].publish(m_est_v[m_index]);
	}
	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
	{
		ros::Time rightnow = ros::Time::now();
		double dt = rightnow.toSec() - m_lpos_time_v[m_index].toSec();
		m_lpos_time_v[m_index] = rightnow;
		m_est_v[m_index].pos_est.x = msg->transform.translation.x;
		m_est_v[m_index].pos_est.y = msg->transform.translation.y;
		m_est_v[m_index].pos_est.z = msg->transform.translation.z;
		m_est_v[m_index].vel_est.x = (m_est_v[m_index].pos_est.x - m_lpos_v[m_index].x)/dt;
		m_est_v[m_index].vel_est.y = (m_est_v[m_index].pos_est.y - m_lpos_v[m_index].y)/dt;
		m_est_v[m_index].vel_est.z = (m_est_v[m_index].pos_est.z - m_lpos_v[m_index].z)/dt;
		m_lpos_v[m_index].x = m_est_v[m_index].pos_est.x;
		m_lpos_v[m_index].y = m_est_v[m_index].pos_est.y;
		m_lpos_v[m_index].z = m_est_v[m_index].pos_est.z;
		
		//TODO
		//m_est_v[vehicle_index].yaw_est = 0;
		m_estpub_v[m_index].publish(m_est_v[m_index]);
	}

	void vicon_est_Callback(geometry_msgs::TransformStamped::ConstPtr& msg) //get the vicon datas and calculate the vehicles.
	{
		float dt = 1.0f/m_vicon_freq;
		
	}
};
void num_veh_Callback(const crazyflie_driver::num_vehiclepub::ConstPtr& msg)
{
	g_vehicle_num = msg->g_vehicle_num;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "linker");
	ros::NodeHandle n("~");
  	ros::Subscriber num_vehiclepub = n.subscribe<crazyflie_driver::num_vehiclepub>("/num_vehiclepub",5,&num_veh_Callback);
	Linker linker(n);
	//int ret = linker.connect_get_param(argc, argv);

	linker.run(50);


  return 0;


}
