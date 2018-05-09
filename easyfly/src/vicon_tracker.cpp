#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Imu.h"
#include <easyfly/pos_est_test.h>	
#include <easyfly/vicon_markernum.h>
#include <easyfly/pos_ctrl_sp.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include "commons.h"
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
#include "type_methode.h"
#include "swarmVel_filter.h"
#include <easyfly/output.h>
//#include "gl_declair.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int g_vehicle_num=2;
const int DimOfVarSpace = 2;

using namespace std;
using namespace cv;
using namespace Eigen;

std::vector<float> yaw_manuel;
bool yaw_manuel_ready;
vector<float> x_marker_init;
vector<float> y_marker_init;
vector<int> index_sequence;
vector<float> x_init_pos;
vector<float> y_init_pos;
float amp_coeff;
float z_ground;

class ViconTracker
{
	//friend class Velocity_filter;
public:
    ViconTracker(ros::NodeHandle& nh)
    :m_vicon_markernumpub(g_vehicle_num)
    ,yaw_bias(g_vehicle_num)//walt
    ,swarm_pos(0)
    ,swarm_vel(g_vehicle_num)
	,swarm_vel_err(g_vehicle_num)
    ,isFirstVicon(true)
    ,m_pos_est_v(g_vehicle_num)
	,m_pos_est_onePt_v(g_vehicle_num)
	,m_swarmFrame(g_vehicle_num)
	,swarm_att_sp(0)

	,swarm_acc_spWd(0)
	,swarm_RotationM(0)
	,swarmFrameIntit(0)
	,vicon_dt(0.0f)
	,ratio(0.3)
	,swarm_err_abs(g_vehicle_num,0.0f)
	//,swarm_vel_filters(g_vehicle_num)
    {
		
        char msg_name[50];
        for(int i=0;i<g_vehicle_num;i++){
			
            sprintf(msg_name, "/vehicle%d/marker_number",i);
			m_vicon_markernumpub[i] = nh.advertise<easyfly::vicon_markernum>(msg_name, 1);
		
			sprintf(msg_name,"/vehicle%d/pos_est_test", i); 
			m_pos_est_v[i] = nh.advertise<easyfly::pos_est_test>(msg_name, 5);

			sprintf(msg_name,"/vehicle%d/pos_est_Onept", i); 
			m_pos_est_onePt_v[i] = nh.advertise<easyfly::pos_est_test>(msg_name, 5);

			sprintf(msg_name,"/vehicle%d/output", i);
			m_output_sub[i] = nh.subscribe<easyfly::output>(msg_name,5,boost::bind(&ViconTracker::output_Cb,this, _1, i));

			/*initialize frame*/	
				for(int index=0;index<4;++index)
				{
					Vector3f tmp;
					tmp.setZero();
					m_swarmFrame[i].pts_relative_buffer.push_back(tmp);
					m_swarmFrame[i].pts_relative.push_back(tmp);
				}
				Vector3f tmp_zero;
	    		tmp_zero(0) = tmp_zero(1) = tmp_zero(2) = 0;
	    		
	    		swarm_pos_predict.push_back(tmp_zero);
				swarm_pos_predict_buffer.push_back(tmp_zero);
	    		swarm_pos_err.push_back(tmp_zero); 

				swarm_pos_shift_buffer.push_back(tmp_zero);
				swarm_pos_shift.push_back(tmp_zero);
				swarm_pos_shift_observ.push_back(tmp_zero);
				swarm_pos_shift_buffer2.push_back(tmp_zero);

				vel_boundary(0) = 0.9f;
				vel_boundary(1) = 0.9f;
				vel_boundary(2) = 0.4f;

			swarm_vel_filters.push_back(new Velocity_filter(swarm_pos_shift_observ[i],swarm_pos_shift[i],swarm_pos_shift_buffer[i]));
        }//i
        m_viconMarkersub = nh.subscribe<vicon_bridge::Markers>("/vicon/markers",5,&ViconTracker::vicon_markerCallback, this);
		
		
    }
    void unite(vector<float> &x_init_pos,vector<float> &y_init_pos,vector<float> &x_marker_pos,vector<float> &y_marker_pos);
    void displayFunc();
	void shift_frame(Vector3f delta,int index);
	void buffer_frame(int index);
	Vector3f onePt_predict(Vector3f& pt,int i);
    static void onMouse(int event, int x, int y, int, void* userInput)
	{
		if (event != EVENT_LBUTTONDOWN && event != EVENT_LBUTTONUP) return;
		int x_world = x - 500;
		int y_world = 500 - y;
		Mat *img = (Mat*)userInput;
		if (event == EVENT_LBUTTONDOWN)
		{
			circle(*img, Point(x, y), 10, Scalar(0, 0, 255));
			imshow("vicon_test", *img);

			float nearest_dist=-1.0f;
			int nearest_index=0;
			for(int i=0;i<x_init_pos.size();i++){
				float sq_dist=sqrt((x_world-x_init_pos[i]*amp_coeff)*(x_world-x_init_pos[i]*amp_coeff)+(y_world-y_init_pos[i]*amp_coeff)*(y_world-y_init_pos[i]*amp_coeff));
				if(sq_dist<nearest_dist||nearest_dist<0){
					nearest_dist=sq_dist;
					nearest_index=i;
				}
			}
			give_index(nearest_index);
		} else if (event == EVENT_LBUTTONUP)
		{
			float nearest_dist=-1.0f;
			int nearest_index=0;
			for(int i=0;i<x_init_pos.size();i++){
				float sq_dist=sqrt((x_world-x_init_pos[i]*amp_coeff)*(x_world-x_init_pos[i]*amp_coeff)+(y_world-y_init_pos[i]*amp_coeff)*(y_world-y_init_pos[i]*amp_coeff));
				if(sq_dist<nearest_dist||nearest_dist<0){
					nearest_dist=sq_dist;
					nearest_index=i;
				}
			}
			float close_len = -1.0f;
			int close_index = -1;//index in x_marker_init
    		for (int j = 0; j < x_marker_init.size(); ++j)//for every x_marker_init
    		{
    			Vector3f tmp_diff;
    			float tmp_len;	  
    			tmp_diff(0) = x_init_pos[nearest_index] - x_marker_init[j];
    			tmp_diff(1) = y_init_pos[nearest_index] - y_marker_init[j];
    			tmp_diff(2) = 0;
    			tmp_len = sqrt(tmp_diff(0)*tmp_diff(0)+tmp_diff(1)*tmp_diff(1));
    			printf("**********%d\n", j);
    			if (tmp_len > VEHICLE_SIZE*1.5)
    				continue; 
    			
    			float tmp = sqrt((x_marker_init[j]*amp_coeff-x_world)*(x_marker_init[j]*amp_coeff-x_world)+(y_marker_init[j]*amp_coeff-y_world)*(y_marker_init[j]*amp_coeff-y_world));
    			if (tmp < close_len || close_len < 0)
    			{
    				close_len = tmp;
    				close_index = j;
    				printf("*******%d\n", j);
    			}		
    		}
    		float x_arrow = x_marker_init[close_index] - x_init_pos[nearest_index]; 
    		float y_arrow = y_marker_init[close_index] - y_init_pos[nearest_index];
    		line(*img,Point(500+x_init_pos[nearest_index]*amp_coeff, 500-y_init_pos[nearest_index]*amp_coeff),Point(500+x_marker_init[close_index]*amp_coeff, 500-y_marker_init[close_index]*amp_coeff),Scalar(0,0,255),5,CV_AA);
    		yaw_manuel.push_back(atan2(y_arrow, x_arrow));
			
			imshow("vicon_test", *img);
		}
	}
    void vicon_markerCallback(const vicon_bridge::Markers::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& est, int vehicle_index);
	void output_Cb(const easyfly::output::ConstPtr& msg, int vehicle_index);
    static void clear_index()
    {
	    index_sequence.clear();
    }
	
    static void give_index(int index)
    {
	    index_sequence.push_back(index);
    }
private:
    ros::Subscriber m_viconMarkersub;
    std::vector<ros::Publisher> m_vicon_markernumpub,m_pos_est_v,m_pos_est_onePt_v;
    std::vector<Vector3f> m_swarm_pos,swarm_pos,_takeoff_Pos;
    std::vector<Vector3f> swarm_pos_predict,swarm_pos_predict_buffer;//records prediction based on last time position, error, and velocity
	std::vector<Vector3f> swarm_pos_err,swarm_vel_err;//records position error for correction
	std::vector<Vector3f> swarm_pos_shift_observ,swarm_pos_shift, swarm_pos_shift_buffer,swarm_pos_shift_buffer2;//records step(only velocity) from last two positions.
	std::vector<Vector3f> swarm_vel; //swarm velocity estimation for consensus control
    std::vector<Vector3f> m_acc;
    std::vector<ros::Subscriber> m_imusub_v,m_output_sub;
    std::vector<vicon_bridge::Marker> m_markers;
    std::vector<float> yaw_bias;//walt
	std::vector<float> swarm_thrust;
    float vicon_dt,ratio;
	Vector3f vel_boundary;
	typedef struct droneFrame { //crazyflie square frame: two steps
    std::vector<Vector3f> pts_relative,pts_relative_buffer;
	Vector3f center,center_buffer;
	}DroneFrame;
	std::vector<float> swarm_err_abs;
	std::vector<DroneFrame> m_swarmFrame,swarmFrameIntit; // swarm_frame
	std::vector<Velocity_filter*> swarm_vel_filters;
	std::vector<Vector3f> swarm_att_spInit,swarm_att_sp,swarm_acc_spWd;
	std::vector<Matrix3f> swarm_RotationM;
	
    easyfly::pos_est_test m_pos_estmsg,posOneptmsg;
	bool isFirstVicon;
    ros::Time m_last_time_vicon;
	ros::Time m_this_time_vicon; 

	//For sequence initialization
	Mat src = Mat(Size(1000,1000), CV_8UC3, Scalar(0));
};

void ViconTracker::unite(vector<float> &x_init_pos,vector<float> &y_init_pos,vector<float> &x_marker_pos,vector<float> &y_marker_pos)
	{
		vector<bool> all_union(x_marker_pos.size(),0);
		
		for(int i=0;i<x_marker_pos.size();++i)//kick out noise
		{
		    int within_circle = 0;
			for(int j=0;j<x_marker_pos.size();++j)
			{
				if(sqrt((x_marker_pos[i]-x_marker_pos[j])*(x_marker_pos[i]-x_marker_pos[j])+(y_marker_pos[i]-y_marker_pos[j])*(y_marker_pos[i]-y_marker_pos[j]))<ABOUT_EDGE)
					++within_circle;
			}
			if(within_circle<3)
				all_union[i]=1;
		}
		for(int i=0;i<x_marker_pos.size();++i)//choose the first point
		{
			if(all_union[i])continue;
			DroneFrame frame; //tmp_frame
			all_union[i]=1;
			vector<int> num_of_point(3,-1);
			vector<float> min_dstc(3,-1);
			num_of_point[0] = i;
		    float temp_dstc;
		    for(int j=1;j<x_marker_pos.size();++j)//find the nearest point
		    {
		    	if(all_union[j])continue;
		    	temp_dstc=(x_marker_pos[i]-x_marker_pos[j])*(x_marker_pos[i]-x_marker_pos[j])+(y_marker_pos[i]-y_marker_pos[j])*(y_marker_pos[i]-y_marker_pos[j]);
		    	if(min_dstc[0]>temp_dstc||min_dstc[0]<0)
		    	{
		    		num_of_point[1]=j;
		    		min_dstc[0]=temp_dstc;
		    	}
		    }
		    all_union[num_of_point[1]]=1;
		    for(int j=1;j<x_marker_pos.size();++j)//find the second nearest point
		    {
		    	if(all_union[j])continue;
		    	temp_dstc=(x_marker_pos[i]-x_marker_pos[j])*(x_marker_pos[i]-x_marker_pos[j])+(y_marker_pos[i]-y_marker_pos[j])*(y_marker_pos[i]-y_marker_pos[j]);
		    	if(min_dstc[1]>temp_dstc||min_dstc[1]<0)
		    	{
		    		num_of_point[2]=j;
		    		min_dstc[1]=temp_dstc;
		    	}
		    }
		    all_union[num_of_point[2]]=1;
		    min_dstc[2]=(x_marker_pos[num_of_point[2]]-x_marker_pos[num_of_point[1]])*(x_marker_pos[num_of_point[2]]-x_marker_pos[num_of_point[1]])+(y_marker_pos[num_of_point[2]]-y_marker_pos[num_of_point[1]])*(y_marker_pos[num_of_point[2]]-y_marker_pos[num_of_point[1]]);
            
            float max_dstc;   
		    int num_of_max_dstc1,num_of_max_dstc2; 
		    if(min_dstc[0]>min_dstc[1])//find the farthest two points  ?? the first condition seems no need
		    {
                num_of_max_dstc1=i;
                num_of_max_dstc2=num_of_point[1];
                max_dstc=min_dstc[0];
		    }
			if(min_dstc[1]>min_dstc[0]){
				int temp_exch;						//find the farthest two points
		    	num_of_max_dstc1=i;
                num_of_max_dstc2=num_of_point[2];
                max_dstc=min_dstc[1];
		    }

		    if(max_dstc<min_dstc[2])
		    {
				int exchg;
		    	num_of_max_dstc1=num_of_point[1];
                num_of_max_dstc2=num_of_point[2];
				exchg = num_of_point[2];
				num_of_point[2] = i; //the third point
				num_of_point[0] = num_of_point[1];
				num_of_point[1] = exchg;
		    }
			temp_dstc=x_marker_pos[num_of_max_dstc1]+x_marker_pos[num_of_max_dstc2];//calculate the centre point
		    x_init_pos.push_back(temp_dstc/2);
		    temp_dstc=y_marker_pos[num_of_max_dstc1]+y_marker_pos[num_of_max_dstc2];
		    y_init_pos.push_back(temp_dstc/2);
			Vector3f center;
			center.setZero();
			center(0) = x_init_pos.back();
			center(1) = y_init_pos.back();
			center(2) = z_ground;
			frame.center_buffer = frame.center = center;
			Vector3f far_pts,relative;
			for(int pt=0;pt<3;++pt){ //three relative pts
				far_pts.setZero();
				relative.setZero();
				far_pts(0) = x_marker_pos[num_of_point[pt]];
				far_pts(1) = y_marker_pos[num_of_point[pt]];
				far_pts(2) = z_ground;
				relative = vec3f_minus(&far_pts,&center);
				frame.pts_relative.push_back(relative); 
				frame.pts_relative_buffer.push_back(relative);
			}
			//relative.setZero();
			//relative = vec3f_minus(&four_pt,&center);
			Vector3f four_pt; //calculate the fourth point
			four_pt.setZero();
			/*four_pt(0) = x_marker_pos[0]+x_marker_pos[1]-x_marker_pos[2];
			four_pt(1) = y_marker_pos[0]+y_marker_pos[1]-y_marker_pos[2];
			four_pt(2) = z_ground;*/
			four_pt(0) = -frame.pts_relative[2](0);
			four_pt(1) = -frame.pts_relative[2](1);
			four_pt(2) = 0.0f;

			frame.pts_relative.push_back(four_pt);
			frame.pts_relative_buffer.push_back(four_pt);
		    for(int j=1;j<x_marker_pos.size();++j)//find and kick out the forth point, if cant find, it doesnt matter
		    {
		    	if(all_union[j])continue;
		    	float cross_product,len_1,len_2;
		    	len_1=sqrt((x_marker_pos[num_of_max_dstc1]-x_marker_pos[j])*(x_marker_pos[num_of_max_dstc1]-x_marker_pos[j])+(y_marker_pos[num_of_max_dstc1]-y_marker_pos[j])*(y_marker_pos[num_of_max_dstc1]-y_marker_pos[j]));
		    	len_2=sqrt((x_marker_pos[num_of_max_dstc2]-x_marker_pos[j])*(x_marker_pos[num_of_max_dstc2]-x_marker_pos[j])+(y_marker_pos[num_of_max_dstc2]-y_marker_pos[j])*(y_marker_pos[num_of_max_dstc2]-y_marker_pos[j]));
		    	cross_product=(x_marker_pos[num_of_max_dstc2]-x_marker_pos[j])*(x_marker_pos[num_of_max_dstc1]-x_marker_pos[j])+(y_marker_pos[num_of_max_dstc2]-y_marker_pos[j])*(y_marker_pos[num_of_max_dstc1]-y_marker_pos[j]);
		    	float cos_degree=cross_product/len_1/len_2;
		    	if(cos_degree<0.08 && cos_degree>-0.08)
		    	{
		    		all_union[j]=1;
		    		break;
		    	}
		    }//find and kick out the forth point, if cant find, it doesnt matter
			swarmFrameIntit.push_back(frame);
		}//choose the first point
	}//end function

void ViconTracker::displayFunc()
{
		float tmp_max = 0;
		for (int i = 0; i < x_init_pos.size(); ++i)
		{
			float tmp = sqrt(x_init_pos[i]*x_init_pos[i]+y_init_pos[i]*y_init_pos[i]);
			if (tmp_max < tmp)
				tmp_max = tmp;	
		}
		amp_coeff = 400.0f/tmp_max;                        
		printf("tmp_max : %f***********amp_coeff : %f\n", tmp_max, amp_coeff);   

		namedWindow("vicon_test");	
		Point p1 = Point(50,50);
		Point p2 = Point(950,950);
		rectangle(src, p1, p2, CV_RGB(0, 0, 255), -1);


		for(int i=0;i<x_marker_init.size();i++){
			circle(src, Point(500+x_marker_init[i]*amp_coeff, 500-y_marker_init[i]*amp_coeff), 2, Scalar(0, 255, 0));  
	    	printf("x1 %d: %f\n", i, x_marker_init[i]);
	    	printf("y1 %d: %f\n", i, y_marker_init[i]);
    	}


		for(int i=0;i<x_init_pos.size();i++){
			circle(src, Point(500+x_init_pos[i]*amp_coeff, 500-y_init_pos[i]*amp_coeff), 2, Scalar(0, 255, 0));  
	    	printf("x%d: %f\n", i, x_init_pos[i]);
	    	printf("y%d: %f\n", i, y_init_pos[i]);
    	}	
		imshow("vicon_test", src);
}	

void ViconTracker::vicon_markerCallback(const vicon_bridge::Markers::ConstPtr& msg)
	{	
		m_markers = msg->markers;
		
		if(isFirstVicon && msg->markers.size() != 0) //first time vicon
		{	
			m_last_time_vicon = ros::Time::now();
			m_this_time_vicon = ros::Time::now();
			for (auto& Marker : m_markers)
    		{		
    			Vector3f pos;
    			pos(0) = Marker.translation.x/1000.0f;
    			pos(1) = Marker.translation.y/1000.0f;
    			pos(2) = Marker.translation.z/1000.0f;

    			x_marker_init.push_back(pos(0));
				y_marker_init.push_back(pos(1));
				if (pos(2) < 0.2)
				{
					z_ground = pos(2);
				}
			}
    		unite(x_init_pos,y_init_pos,x_marker_init,y_marker_init);//identify crazyflies and get their position into swarm_pos
			bool sequenceIsOk = false;
			while(!sequenceIsOk)//use mouse to rearrange index of swarm_pos
			{
				displayFunc();
				setMouseCallback("vicon_test", onMouse, &src);
				waitKey();
				destroyWindow("vicon_test");
				//printf("%d\n", index_sequence.size());
				/*check the click times and exit the initialization*/
				if(index_sequence.size()==g_vehicle_num){
					sequenceIsOk = true;
					for(int k=0;k<g_vehicle_num;++k){
						Vector3f tem_att;
						tem_att(2) = yaw_manuel[k];
						swarm_att_spInit.push_back(tem_att);

						tem_att.setZero();
						swarm_att_sp.push_back(tem_att);
						swarm_acc_spWd.push_back(tem_att);
						Matrix3f temp_zero;
						temp_zero.setZero();
						swarm_RotationM.push_back(temp_zero);
					}
					
				}else{
					printf("Initialization fails!! Please click again!!\n");
					fflush(stdout);
					clear_index();
					swarm_att_sp.clear();
					swarm_att_spInit.clear();
					swarm_RotationM.clear();
					swarm_acc_spWd.clear();
				}
			}
			yaw_manuel_ready=true;
			
			for (int i=0; i<index_sequence.size();i++)
			{
				Vector3f tmp_pos;
				tmp_pos(0) = x_init_pos[index_sequence[i]];
				tmp_pos(1) = y_init_pos[index_sequence[i]];
				tmp_pos(2) = z_ground;
				swarm_pos.push_back(tmp_pos);
				m_swarmFrame[i] = swarmFrameIntit[index_sequence[i]]; //re-order the frames
				swarm_att_sp[i] = swarm_att_spInit[index_sequence[i]];
				//m_hover_pos.push_back(tmp_pos);
				m_swarm_pos.push_back(tmp_pos);
				_takeoff_Pos.push_back(tmp_pos);
			}

    		if(swarm_pos.size()==g_vehicle_num)
    			isFirstVicon = false;
    	}
		else if(!isFirstVicon && msg->markers.size() != 0)
		{
			std::vector<Vector3f> consider_pos;
			for (auto& Marker : m_markers)//push back markers_pos points;
	    	{	
	    		Vector3f pos;
	    		pos(0) = Marker.translation.x/1000.0f;
	    		pos(1) = Marker.translation.y/1000.0f;
	    		pos(2) = Marker.translation.z/1000.0f;
	    		consider_pos.push_back(pos);
	    	}
			/*find vehicles*/
	    	for (int i = 0; i < g_vehicle_num; ++i)//for every vehicles
	    	{
	    		/*prediction*/
	    		//swarm_pos_predict[i] = swarm_pos[i] + swarm_pos_shift_observ[i] + REVISE_WEIGHT*swarm_pos_err[i]/(vicon_dt+REVISE_WEIGHT);
				swarm_pos_predict[i] = swarm_pos[i] + swarm_pos_shift[i] + swarm_acc_spWd[i]*vicon_dt*vicon_dt/2.0f + REVISE_WEIGHT*swarm_pos_err[i]/(REVISE_WEIGHT+swarm_err_abs[i]);
				shift_frame(swarm_pos_shift[i],i);
	    		/*small wipe out*/

	    		std::vector<Vector3f> close_points;
	    		for (int j = 0; j < consider_pos.size(); ++j)//push back close ploints
	    		{
	    			Vector3f tmp_diff;
	    			float tmp_norm;	  
	    			tmp_diff(0) = consider_pos[j](0) - swarm_pos_predict[i](0);
	    			tmp_diff(1) = consider_pos[j](1) - swarm_pos_predict[i](1);
	    			tmp_diff(2) = consider_pos[j](2) - swarm_pos_predict[i](2);
	    			vec3f_norm(&tmp_diff, &tmp_norm);

	    			//if (tmp_norm < VEHICLE_SIZE)
					if (tmp_norm < VEHICLE_SIZE*1.5)
	    				close_points.push_back(consider_pos[j]);  			
	    		}//j

				/*message number of markers detected*/
	    		easyfly::vicon_markernum mk_num_msg;
	    		mk_num_msg.vicon_marker_number = close_points.size();
	    		m_vicon_markernumpub[i].publish(mk_num_msg);

				/*for comparison of one point prediction*/
				if(close_points.size()>0){
					int nearest_index;
					float dot,dist_newrelate;
					dot=dist_newrelate=0.0f;
	
				for(int k=0;k<close_points.size();++k){
					Vector3f new_relative;
					new_relative.setZero();
					new_relative = vec3f_minus(&close_points[k],&m_swarmFrame[i].center);
					float len1,len2;
					vec3f_norm(&new_relative,&len1);
					vec3f_norm(&m_swarmFrame[i].pts_relative[0],&len2);
					float costheta = vec3f_dot(&new_relative,&m_swarmFrame[i].pts_relative[0])/len1/len2;
					if(costheta>dot){
						nearest_index = k;
						dot = costheta;
					}
				}
				/*posOneptmsg.pos_est.x = close_points[nearest_index](0);
				posOneptmsg.pos_est.y = close_points[nearest_index](1);
				posOneptmsg.pos_est.z = close_points[nearest_index](2);*/
				}
				
				/*m_pos_est_onePt_v[i].publish(posOneptmsg);*/

				/*condition 1 to 4*/
	    		if (close_points.size() >= 4)// && close_points.size() <= 4*g_vehicle_num) //condition 4
	    		{
	    			bool FoundVehicle_i = false;
					/*find vehicle center from close_points*/
		    		for (int j = 0; j < close_points.size(); ++j)
		    		{
		    			if (FoundVehicle_i)
		    				break;
		    			/*record all the vectors that based on points j*/
		    			std::vector<Vector3f> consider_vec; 
		    			for (int k = 0; k < close_points.size(); ++k)
		    			{
		    				if (k != j)
		    				{
		    					Vector3f tmp_vec;
								tmp_vec = vec3f_minus(&close_points[k],&close_points[j]);
			    				consider_vec.push_back(tmp_vec);
		    				}
		    			}
		    			/*count the number of right pairs*/
		    			for (int p = 0; p < consider_vec.size(); ++p)
		    			{
		    				std::vector<Vector3f> swarm_pos_p;
		    				int count_p = 0;
		    				float len_p; 
		    				vec3f_norm(&consider_vec[p], &len_p);
		    				for (int q = 0; q < consider_vec.size(); ++q)
		    				{
		    					if (q != p)
		    					{
				    				float len_q;
				    				vec3f_norm(&consider_vec[q], &len_q);
				    				float ctheta = (consider_vec[p](0)*consider_vec[q](0)+consider_vec[p](1)*consider_vec[q](1)+consider_vec[p](2)*consider_vec[q](2))/(len_p*len_q);
									if (ctheta < 0.2 && len_q/len_p < 1.05 && len_q/len_p > 0.95)
			    					{
			    						//printf("condition 1\n");
			    						Vector3f tmp_pos;
			    						tmp_pos(0) = 0.5*(consider_vec[p](0) + consider_vec[q](0)) + close_points[j](0);
			    						tmp_pos(1) = 0.5*(consider_vec[p](1) + consider_vec[q](1)) + close_points[j](1);
			    						tmp_pos(2) = 0.5*(consider_vec[p](2) + consider_vec[q](2)) + close_points[j](2);
			    						swarm_pos_p.push_back(tmp_pos);
			    						count_p++;
			    					} else if (ctheta < 0.75 && ctheta > 0.65 && len_q/len_p < 1.45 && len_q/len_p > 1.35)
			    					{
			    						//printf("condition 2\n");
			    						Vector3f tmp_pos;
			    						tmp_pos(0) = 0.5*consider_vec[q](0) + close_points[j](0);
			    						tmp_pos(1) = 0.5*consider_vec[q](1) + close_points[j](1);
			    						tmp_pos(2) = 0.5*consider_vec[q](2) + close_points[j](2);
			    						swarm_pos_p.push_back(tmp_pos);
			    						count_p++;
			    					} else if (ctheta < 0.75 && ctheta > 0.65 && len_p/len_q < 1.45 && len_p/len_q > 1.35)
			    					{
			    						//printf("condition 3\n");
			    						Vector3f tmp_pos;
			    						tmp_pos(0) = 0.5*consider_vec[p](0) + close_points[j](0);
			    						tmp_pos(1) = 0.5*consider_vec[p](1) + close_points[j](1);
			    						tmp_pos(2) = 0.5*consider_vec[p](2) + close_points[j](2);
			    						swarm_pos_p.push_back(tmp_pos);
			    						count_p++;
			    					}
		    					}//if
		    				}//for q
		    				if (count_p == 2)
		    				{
		    					m_swarm_pos[i](0) = (swarm_pos_p[0](0) + swarm_pos_p[1](0))/2;
		    					m_swarm_pos[i](1) = (swarm_pos_p[0](1) + swarm_pos_p[1](1))/2;
		    					m_swarm_pos[i](2) = (swarm_pos_p[0](2) + swarm_pos_p[1](2))/2;
		    					FoundVehicle_i = true;
		    					break;
		    				} else if (count_p == 1)
		    				{
		    					m_swarm_pos[i] = swarm_pos_p[0];
		    					FoundVehicle_i = true;
		    					break;
		    				}		
		    			}//for p
		    		}//for j
		    		if (!FoundVehicle_i)
    				{
    				printf("*****condition 4 failed! failure number : 1\n");
    				}
	    		}else if (close_points.size() == 3) //condition 3
	    		{
	    			//printf("*****condition 3\n");
	    			Vector3f tmp_vec_1;
	    			float tmp_len_1;
	    			tmp_vec_1(0) = close_points[1](0) - close_points[0](0);
	    			tmp_vec_1(1) = close_points[1](1) - close_points[0](1);
	    			tmp_vec_1(2) = close_points[1](2) - close_points[0](2);
	    			vec3f_norm(&tmp_vec_1, &tmp_len_1);

	    			Vector3f tmp_vec_2;
	    			float tmp_len_2;
	    			tmp_vec_2(0) = close_points[2](0) - close_points[0](0);
	    			tmp_vec_2(1) = close_points[2](1) - close_points[0](1);
	    			tmp_vec_2(2) = close_points[2](2) - close_points[0](2);
	    			vec3f_norm(&tmp_vec_2, &tmp_len_2);
	    			
	    			float ctheta = (tmp_vec_1(0)*tmp_vec_2(0)+tmp_vec_1(1)*tmp_vec_2(1)+tmp_vec_1(2)*tmp_vec_2(2))/(tmp_len_1*tmp_len_2);
	    			if (ctheta < 0.75 && ctheta > 0.65)
	    			{
	    				if (tmp_len_2/tmp_len_1 < 1.5 && tmp_len_2/tmp_len_1 > 1.3)
	    				{
	    					Vector3f tmp;
			    			tmp(0) = 0.5*(close_points[2](0) + close_points[0](0));
			    			tmp(1) = 0.5*(close_points[2](1) + close_points[0](1));
			    			tmp(2) = 0.5*(close_points[2](2) + close_points[0](2));	
			    			m_swarm_pos[i] = tmp;
	    				} else if (tmp_len_1/tmp_len_2 < 1.5 && tmp_len_1/tmp_len_2 > 1.3)
	    				{
	    					Vector3f tmp;
			    			tmp(0) = 0.5*(close_points[1](0) + close_points[0](0));
			    			tmp(1) = 0.5*(close_points[1](1) + close_points[0](1));
			    			tmp(2) = 0.5*(close_points[1](2) + close_points[0](2));	
			    			m_swarm_pos[i] = tmp;
	    				} else{
	    					printf("*****condition 3 failed! failure number : 1\n");
	    				}
	    			} else if (ctheta < 0.1 && fabs(tmp_len_2-tmp_len_1)<0.2)
	    			{
	    				Vector3f tmp;
			    		tmp(0) = 0.5*(close_points[2](0) + close_points[1](0));
			    		tmp(1) = 0.5*(close_points[2](1) + close_points[1](1));
			    		tmp(2) = 0.5*(close_points[2](2) + close_points[1](2));	
			    		m_swarm_pos[i] = tmp;
	    			} else {
	    				printf("*****condition 3 failed! failure number : 2\n");
	    			}
	    		} else if (close_points.size() == 2) //condition 2
	    		{
	    			//printf("*****condition 2\n");
    				Vector3f tmp_vec;
    				float tmp_len;
					bool isRdge = false;
		    		tmp_vec(0) = close_points[1](0) - close_points[0](0);
		    		tmp_vec(1) = close_points[1](1) - close_points[0](1);
		    		tmp_vec(2) = close_points[1](2) - close_points[0](2);	
		    		vec3f_norm(&tmp_vec, &tmp_len);
					Vector3f center_pos;
					//printf("----------- lenth between 2 points:  %f ---------------\n",tmp_len);
		    		if (tmp_len > VEHICLE_SIZE*0.8 && tmp_len < VEHICLE_SIZE*1.2) //diagonal
		    		{
		    			center_pos(0) = 0.5*(close_points[1](0) + close_points[0](0));
			    		center_pos(1) = 0.5*(close_points[1](1) + close_points[0](1));
			    		center_pos(2) = 0.5*(close_points[1](2) + close_points[0](2));	
						isRdge = true;
			    		
		    		} else if (tmp_len < VEHICLE_SIZE/1.414*1.2 && tmp_len > VEHICLE_SIZE/1.414*0.8) //edge
		    		{
		    			Vector3f ctr_pdct1 = onePt_predict(close_points[0],i);
						Vector3f ctr_pdct2 = onePt_predict(close_points[1],i);
						for(int m=0;m<3;++m){
							center_pos(m) = (ctr_pdct1(m)+ctr_pdct2(m))/2.0f;
						}
						isRdge = true; 
		    		}
					if(isRdge){
						Vector3f predict_diff;
			    		predict_diff(0) = swarm_pos_predict[i](0) - center_pos(0);
			    		predict_diff(1) = swarm_pos_predict[i](1) - center_pos(1);
			    		predict_diff(2) = swarm_pos_predict[i](2) - center_pos(2);
			    		float tmp_dist;
			    		vec3f_norm(&predict_diff, &tmp_dist);
			    		ratio = 0.3/(0.3+tmp_dist);
			    		Vector3f tmp;
						tmp(0) = center_pos(0);
			    		tmp(1) = center_pos(1);// + ratio*swarm_pos_predict[0](1);
			    		tmp(2) = center_pos(2);// + ratio*swarm_pos_predict[0](2);	
			    		m_swarm_pos[i] = tmp;
					}
					
					else {
		    			printf("*****condition 2 failed! failure number : 0\n");
		    		}

	    		} else if (close_points.size() == 1)
	    		{
	    			//printf("*****condition 1\n");
	    			Vector3f predict_diff;
		    		predict_diff(0) = swarm_pos_predict[i](0) - close_points[0](0);
		    		predict_diff(1) = swarm_pos_predict[i](1) - close_points[0](1);
		    		predict_diff(2) = swarm_pos_predict[i](2) - close_points[0](2);
		    		float tmp_dist;
		    		vec3f_norm(&predict_diff, &tmp_dist);
					Vector3f ctr_pdct;
					ctr_pdct = onePt_predict(close_points[0],i); //predict center using onr point and the frame
		    		ratio = 0.3/(0.3+tmp_dist);
		    		Vector3f tmp;
		    		//tmp(0) = (1-ratio)*ctr_pdct(0);// + ratio*swarm_pos_predict[0](0);
					tmp(0) = ctr_pdct(0);
		    		tmp(1) = ctr_pdct(1);// + ratio*swarm_pos_predict[0](1);
		    		tmp(2) = ctr_pdct(2);// + ratio*swarm_pos_predict[0](2);	
		    		m_swarm_pos[i] = tmp;
	    		} else {
	    			printf("*****Cannot find vehicle%d\n", i);
	    			continue;
	    		}				
	    		/*renew error*/
	    		swarm_pos_err[i](0) = m_swarm_pos[i](0) - swarm_pos_predict[i](0);
	    		swarm_pos_err[i](1) = m_swarm_pos[i](1) - swarm_pos_predict[i](1);
	    		swarm_pos_err[i](2) = m_swarm_pos[i](2) - swarm_pos_predict[i](2);
				buffer_frame(i);
				swarm_pos_shift_observ[i](0) = m_swarm_pos[i](0) - swarm_pos[i](0);
	    		swarm_pos_shift_observ[i](1) = m_swarm_pos[i](1) - swarm_pos[i](1);
	    		swarm_pos_shift_observ[i](2) = m_swarm_pos[i](2) - swarm_pos[i](2);

				swarm_pos_predict_buffer[i] = swarm_pos_predict[i];
				vec3f_norm(&swarm_pos_err[i], &swarm_err_abs[i]);
				printf("--------- err_abs %d : %f ------------\n",i,swarm_err_abs[i]);

				/*LP filter*/
				swarm_vel_filters[i]->update(swarm_pos_shift_observ[i]);
				swarm_pos_shift[i] = swarm_vel_filters[i]->filter_vec3f();
				
				/*Average filter*/
				swarm_pos_shift[i] = swarm_pos_shift_observ[i];
				
	    	}//i

			/*renew and publish*/
			m_this_time_vicon = ros::Time::now();
			vicon_dt = (m_this_time_vicon - m_last_time_vicon).toSec();
			m_last_time_vicon = m_this_time_vicon;
			m_pos_estmsg.header.stamp = ros::Time::now();
	    	for (int i = 0; i < swarm_pos.size(); ++i)
	    	{
				/*boundary condition*/
				for(int j=0;j<3;++j){
					if(fabs(swarm_pos_shift[i](j)/vicon_dt)>vel_boundary(j)){
						if(swarm_pos_shift[i](j)>0)swarm_pos_shift[i](j) = vel_boundary(j)*vicon_dt;
						else swarm_pos_shift[i](j) = -vel_boundary(j)*vicon_dt;
						}
					swarm_pos_shift[i](j) = (swarm_pos_shift[i](j) + swarm_pos_shift_buffer2[i](j) + swarm_pos_shift_buffer[i](j))/3.0f;
				}
				swarm_pos_shift_buffer2[i] = swarm_pos_shift_buffer[i];
				swarm_pos_shift_buffer[i] = swarm_pos_shift[i];
				
	    		swarm_pos[i] = m_swarm_pos[i];
	    		m_pos_estmsg.pos_est.x = swarm_pos[i](0);
				m_pos_estmsg.pos_est.y = swarm_pos[i](1);
				m_pos_estmsg.pos_est.z = swarm_pos[i](2);
				m_pos_estmsg.vehicle_index = i;
				float vx,vy,vz;
				vx=vy=vz=0.0f;
				vx = swarm_pos_shift[i](0)/vicon_dt;
				vy = swarm_pos_shift[i](1)/vicon_dt;
				vz = swarm_pos_shift[i](2)/vicon_dt;

				m_pos_estmsg.vel_est.x = vx;
				m_pos_estmsg.vel_est.y = vy;
				m_pos_estmsg.vel_est.z = vz;

				vx = swarm_pos_shift_observ[i](0)/vicon_dt;
				vy = swarm_pos_shift_observ[i](1)/vicon_dt;
				vz = swarm_pos_shift_observ[i](2)/vicon_dt;
				
				posOneptmsg.vel_est.x = vx;
				posOneptmsg.vel_est.y = vy;
				posOneptmsg.vel_est.z = vz;

				std::vector<float> alpha_v = swarm_vel_filters[i]->getAlpha_v();
				m_pos_estmsg.alpha_v.x = alpha_v[0];
				m_pos_estmsg.alpha_v.y = alpha_v[1];
				m_pos_estmsg.alpha_v.z = alpha_v[2];

				/*printf("------------------ time %f ------------------\n",vicon_dt);
				fflush(stdout);*/
	    		m_pos_est_v[i].publish(m_pos_estmsg);
				m_pos_est_onePt_v[i].publish(posOneptmsg);
	    	}
	    	
		}// if !isFirstVicon && msg->markers.size() != 0  	
	}
void ViconTracker::imuCallback(const sensor_msgs::Imu::ConstPtr& est, int vehicle_index)
	{
		m_acc[vehicle_index](0) = est->linear_acceleration.x;
        m_acc[vehicle_index](1) = est->linear_acceleration.y;
        m_acc[vehicle_index](2) = est->linear_acceleration.z;
    }
void ViconTracker::output_Cb(const easyfly::output::ConstPtr& msg, int vehicle_index)
{
	swarm_att_sp[vehicle_index](0) = msg->att_sp.x;
	swarm_att_sp[vehicle_index](1) = msg->att_sp.y;
	euler2rotation(&swarm_att_sp[vehicle_index],&swarm_RotationM[vehicle_index]);
	Vector3f tmp_acc_Bd;
	tmp_acc_Bd.setZero();
	tmp_acc_Bd(2) = msg->throttle * 470.0f / VEHICLE_MASS;
	body2earth(&swarm_RotationM[vehicle_index],&tmp_acc_Bd,&swarm_acc_spWd[vehicle_index],3);
}
void ViconTracker::shift_frame(Vector3f delta,const int index)
{
	m_swarmFrame[index].center = vec3f_shift(&m_swarmFrame[index].center_buffer,&delta); 	
}

/*void update_relative()
{
	for(int n=0;n<4;++n)
	{
		m_swarmFrame[index].pts_relative[n] = vec3f_shift(&m_swarmFrame[index].pts_relative_buffer[n],&delta);
		Vector3f tmpres = vec3f_minus(&m_swarmFrame[index].pts_relative[n],&m_swarmFrame[index].pts_relative_buffer[n]);
		//if(tmpres(0) == 0.0f)printf("Frame not moving!!!!!\n");
	}
}*/
void ViconTracker::buffer_frame(const int index)
{
	m_swarmFrame[index].center_buffer = m_swarmFrame[index].center;
	/*for(int i=0;i<4;++i)
	{
		m_swarmFrame[index].pts_relative_buffer[i] = m_swarmFrame[index].pts_relative[i];
	}*/
}
Vector3f ViconTracker::onePt_predict(Vector3f& pt,const int i)
{
	int nearest_index;
	float dot,dist_newrelate;
	dot=dist_newrelate=0.0f;
	Vector3f new_relative;
	new_relative.setZero();
	new_relative = vec3f_minus(&pt,&m_swarmFrame[i].center);
	
	for(int pt_relative=0;pt_relative<4;++pt_relative)
	{	
		//printf("****************new relative %f   %f   %f ********************\n",new_relative(0),new_relative(1),new_relative(2));
		//printf("---------------- DEBUG %f----------------------\n",vec3f_dot(&new_relative,&m_swarmFrame[i].pts_relative[pt_relative]));
		float len1,len2;
		vec3f_norm(&new_relative,&len1);
		vec3f_norm(&m_swarmFrame[i].pts_relative[pt_relative],&len2);
		float costheta = vec3f_dot(&new_relative,&m_swarmFrame[i].pts_relative[pt_relative])/len1/len2;
		if(costheta>dot){
			nearest_index = pt_relative;
			//dot = vec3f_dot(&new_relative,&m_swarmFrame[i].pts_relative[pt_relative]);
			dot = costheta;
		}
	}
	/*Vector3f del_tmp;
	del_tmp.setZero();
	del_tmp = close_points[0] - (m_swarmFrame[i].center + m_swarmFrame[i].pts_relative[nearest_index]);
	close_points[0] = vec3f_shift(close_points[0],del_tmp);*/
	
	return vec3f_minus(&pt,&m_swarmFrame[i].pts_relative[nearest_index]); //predicted center using one point
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_tracker");
	ros::NodeHandle n;
	
	n.getParam("/g_vehicle_num", g_vehicle_num);
	
	ViconTracker vicon_tracker(n);
	ros::spin();
	
    return 0;
}