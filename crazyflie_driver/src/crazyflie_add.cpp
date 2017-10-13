#include "ros/ros.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include <crazyflie_driver/num_vehiclepub.h>
#include <stdio.h> //sprintf

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_add", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");
  int g_vehicle_num;
  
  std::string uri;
  std::string  tf_prefix;
  float roll_trim;
  float pitch_trim;
  int group_index;

  bool enable_logging;
  bool enable_parameters;
  bool use_ros_time;
  bool enable_logging_imu;
  bool enable_logging_temperature;
  bool enable_logging_magnetic_field;
  bool enable_logging_pressure;
  bool enable_logging_battery;
  //int g_joy_num;

  n.getParam("g_vehicle_num",g_vehicle_num);

  //char param_name[50];
  //load params under the specific namespace
  n.getParam("group_index", group_index);
  n.getParam("uri", uri);
  n.getParam("tf_prefix", tf_prefix);
  n.getParam("roll_trim", roll_trim);
  n.getParam("pitch_trim", pitch_trim);
  n.getParam("enable_logging_temperature", enable_logging_temperature);

  n.param("enable_logging", enable_logging,true);
  n.param("enable_parameters", enable_parameters,true);
  n.param("use_ros_time", use_ros_time,true);
  n.param("enable_logging_imu", enable_logging_imu,true);
  //n.param("enable_logging_temperature", enable_logging_temperature,true);
  n.param("enable_logging_magnetic_field", enable_logging_magnetic_field,true);
  n.param("enable_logging_pressure", enable_logging_pressure,true);
  n.param("enable_logging_battery", enable_logging_battery,true);

//publish the number of vehicles
  ros::NodeHandle n_numv;
  char msg_name[50];
  //sprintf(msg_name,"num_vehiclepub", g_vehicle_num);
  sprintf(msg_name,"/num_vehiclepub");
  ros::Publisher num_vehiclepub;
  crazyflie_driver::num_vehiclepub num_veh_msg;
  num_vehiclepub = n_numv.advertise<crazyflie_driver::num_vehiclepub>(msg_name,1);
  num_veh_msg.g_vehicle_num=g_vehicle_num;
  num_vehiclepub.publish(num_veh_msg); 

  ROS_INFO("wait_for_service add_crazyflie");
  char servicename[50];
  //sprintf(servicename,"/vehicle%d/add_crazyflie",group_index);
  ros::ServiceClient addCrazyflieService = n.serviceClient<crazyflie_driver::AddCrazyflie>("/add_crazyflie"); //client instance
  addCrazyflieService.waitForExistence();
  //ROS_INFO("found%s",servicename);
  ROS_INFO("found%s","/add_crazyflie");
  
    crazyflie_driver::AddCrazyflie addCrazyflie; //containt of req and res

    addCrazyflie.request.g_vehicle_num = g_vehicle_num;
    addCrazyflie.request.group_index = group_index;
    addCrazyflie.request.uri = uri;
    addCrazyflie.request.tf_prefix = tf_prefix;
    addCrazyflie.request.roll_trim = roll_trim;
    addCrazyflie.request.pitch_trim = pitch_trim;
    addCrazyflie.request.enable_logging = enable_logging;
    addCrazyflie.request.enable_parameters = enable_parameters;
    addCrazyflie.request.use_ros_time = use_ros_time;
    addCrazyflie.request.enable_logging_imu = enable_logging_imu;
    addCrazyflie.request.enable_logging_temperature = enable_logging_temperature;
    addCrazyflie.request.enable_logging_magnetic_field = enable_logging_magnetic_field;
    addCrazyflie.request.enable_logging_pressure = enable_logging_pressure;
    addCrazyflie.request.enable_logging_battery = enable_logging_battery;

  std::vector<std::string> genericLogTopics;
  n.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
  std::vector<int> genericLogTopicFrequencies;
  n.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>()); //till here OK
  
  if (genericLogTopics.size() == genericLogTopicFrequencies.size())
  {

    size_t i = 0;
    for (auto& topic : genericLogTopics)
    {
      crazyflie_driver::LogBlock logBlock;
      logBlock.topic_name = topic;
      logBlock.frequency = genericLogTopicFrequencies[i];
      n.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
      addCrazyflie.request.log_blocks.push_back(logBlock);
      ++i;
    }
  }
  else
  {
    ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
  }

  addCrazyflieService.call(addCrazyflie);
  
  return 0;
}
