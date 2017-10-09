#include "ros/ros.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include <stdio.h> //sprintf

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_add", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");
  int g_vehicle_num;
  // read paramaters
  std::vector<std::string> uri_v;
  std::vector<std::string> tf_prefix_v;
  std::vector<float> roll_trim_v;
  std::vector<float> pitch_trim_v;
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
  char param_name[50];
  //n.getParam("g_joy_num",g_joy_num);
  for (int i=0;i<g_vehicle_num;i++){
    sprintf(param_name,"/vehicle%d/uri",i);
    n.getParam(param_name, uri_v[i]);
    sprintf(param_name,"/vehicle%d/tf_prefix",i);
    n.getParam(param_name, tf_prefix_v[i]);
    sprintf(param_name,"/vehicle%d/roll_trim",i);
    n.getParam(param_name, roll_trim_v[i]);
    sprintf(param_name,"/vehicle%d/pitch_trim",i);
    n.getParam(param_name, pitch_trim_v[i]);
}
    n.param("enable_logging", enable_logging,true);
    n.param("enable_parameters", enable_parameters,true);
    n.param("use_ros_time", use_ros_time,true);
    n.param("enable_logging_imu", enable_logging_imu,true);
    n.param("enable_logging_temperature", enable_logging_temperature,true);
    n.param("enable_logging_magnetic_field", enable_logging_magnetic_field,true);
    n.param("enable_logging_pressure", enable_logging_pressure,true);
    n.param("enable_logging_battery", enable_logging_battery,true);

  ROS_INFO("wait_for_service /add_crazyflie");
  ros::ServiceClient addCrazyflieService = n.serviceClient<crazyflie_driver::AddCrazyflie>("/add_crazyflie");
  addCrazyflieService.waitForExistence();
  ROS_INFO("found /add_crazyflie");
  crazyflie_driver::AddCrazyflie addCrazyflie;
  addCrazyflie.request.g_vehicle_num = g_vehicle_num;
  addCrazyflie.request.uri_v = uri_v;
  addCrazyflie.request.tf_prefix_v = tf_prefix_v;
  addCrazyflie.request.roll_trim_v = roll_trim_v;
  addCrazyflie.request.pitch_trim_v = pitch_trim_v;
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
  n.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>());

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
