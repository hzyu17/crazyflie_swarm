#include "ros/ros.h"
#include "crazyflie_driver/Att_est.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "commons.h"
#include <crazyflie_driver/num_vehiclepub.h>
#include "std_msgs/Float32.h"
#include <stdio.h> //sprintf
#include <easyfly/output.h>
#include <thread>
#include <mutex>
#include "attitude_estimation.h"
#include <crazyflie_cpp/Crazyflie.h>

using namespace Eigen;

/*const float K_Pa = 0.5f;
const float K_Ia = 0.1f;
const float K_Pm = 0.3f;
const float K_Im = 0.1f;*/

//constexpr double pi() { return std::atan(1)*4; }


class CrazyflieROS : private Attitude_estimator 
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    int group_index,
    float roll_trim,
    float pitch_trim,
    bool enable_logging,
    bool enable_parameters,
    std::vector<crazyflie_driver::LogBlock>& log_blocks,
    bool use_ros_time,
    bool enable_logging_imu,
    bool enable_logging_temperature,
    bool enable_logging_magnetic_field,
    bool enable_logging_pressure,
    bool enable_logging_battery)
    : m_cf(link_uri)
    , m_uri(link_uri)
    , m_tf_prefix(tf_prefix)
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_enableLogging(enable_logging)
    , m_enableParameters(enable_parameters)
    , m_logBlocks(log_blocks)
    , m_use_ros_time(use_ros_time)
    , m_enable_logging_imu(enable_logging_imu)
    , m_enable_logging_temperature(enable_logging_temperature)
    , m_enable_logging_magnetic_field(enable_logging_magnetic_field)
    , m_enable_logging_pressure(enable_logging_pressure)
    , m_enable_logging_battery(enable_logging_battery)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_subscribeCmdVel()
    , m_pubImu() //inertial measurement unit
    , m_pubTemp()
    , m_pubMag()
    , m_pubPressure()
    , m_pubBattery()
    , m_pubRssi() //Received Signal Strength Indication
    , m_sentSetpoint(false)
    , m_group_index(group_index)
    , m_att()
    , m_acc()
    , m_mag()
    , m_gyro()
    , m_init_R(true)
  {
    char msg_name[50];
    m_uri = link_uri;
    ros::NodeHandle n("~");
    ros::NodeHandle nh;
    sprintf(msg_name,"/vehicle%d/att_est",m_group_index);
    m_yawpub = n.advertise<crazyflie_driver::Att_est>(msg_name,5);
    m_prevsT_integ_err = ros::Time::now();

    m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
    m_serviceUpdateParams = n.advertiseService(tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);

    sprintf(msg_name,"/vehicle%d/output",m_group_index);
    m_outputsub = nh.subscribe<easyfly::output>(msg_name,5,&CrazyflieROS::outputCallback, this);

    if (m_enable_logging_imu) {
    	sprintf(msg_name,"/vehicle%d/tf_prefix/Imu",m_group_index);
    	m_pubImu = n.advertise<sensor_msgs::Imu>(msg_name, 10);
    }
    if (m_enable_logging_temperature) {
    	sprintf(msg_name,"/vehicle%d/Temperature",m_group_index);
      m_pubTemp = n.advertise<sensor_msgs::Temperature>(msg_name, 10);
    }
    if (m_enable_logging_magnetic_field) {
    	sprintf(msg_name,"/vehicle%d/magnetic_field",m_group_index);
      m_pubMag = n.advertise<sensor_msgs::MagneticField>(msg_name, 10);
    }
    if (m_enable_logging_pressure) {
    	sprintf(msg_name,"/vehicle%d/pressure",m_group_index);
      m_pubPressure = n.advertise<std_msgs::Float32>(msg_name, 10);
    }
    if (m_enable_logging_battery) {
    	sprintf(msg_name,"/vehicle%d/battery",m_group_index);
      m_pubBattery = n.advertise<std_msgs::Float32>(msg_name, 10);
    }
    sprintf(msg_name,"/vehicle%d/rssi",m_group_index);
    m_pubRssi = n.advertise<std_msgs::Float32>(msg_name, 10);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
    }
    msg_yaw_est.uri = m_uri;
    msg_yaw_est.group_index = m_group_index;

    msg_yaw_est.att_est.x = m_roll_trim;
    msg_yaw_est.att_est.y = m_pitch_trim;
    msg_yaw_est.att_est.z = 0.0f;
    
    std::thread t(&CrazyflieROS::run, this);
    t.detach();
  }

private:

  struct logImu {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } __attribute__((packed));

  struct log2 {
    float mag_x;
    float mag_y;
    float mag_z;
    float baro_temp;
    float baro_pressure;
    float pm_vbat;
  } __attribute__((packed));

  struct logStablizer {
    float roll;
    float pitch;
    float yaw;
    uint16_t thrust;
  } __attribute__((packed));
  

private:
	void outputCallback(const easyfly::output::ConstPtr& msg)
	{
		m_output.att_sp.x = msg->att_sp.x;
		m_output.att_sp.y = msg->att_sp.y;
		m_output.att_sp.z = msg->att_sp.z;
		m_output.throttle = msg->throttle;
        m_cf.sendSetpoint(m_output.att_sp.x * RAD2DEG,
				m_output.att_sp.y * RAD2DEG,
				m_output.att_sp.z * RAD2DEG,
				m_output.throttle * 65000);
        m_sentSetpoint = true;
        printf("m_output.throttle:  %f\n", m_output.throttle);
	}	
  	bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("Update parameters");
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    return true;
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      float roll = msg->linear.y + m_roll_trim;
      float pitch = - (msg->linear.x + m_pitch_trim);
      float yawrate = msg->angular.z;
      uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
      m_sentSetpoint = true;
    }
  }

  void run()
  {
    // m_cf.reboot();
    m_cf.logReset();
    float frequency = 50;
    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);
    auto start = std::chrono::system_clock::now();

    if (m_enableParameters)
    {
      ROS_INFO("Requesting parameters...");
      m_cf.requestParamToc();
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
    }

    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    std::unique_ptr<LogBlock<logStablizer> > logblockStablizer;
    std::unique_ptr<LogBlock<log2> > logBlock2;
    std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());
    

    if (m_enableLogging) {

      std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
      m_cf.setEmptyAckCallback(cb_ack);

      ROS_INFO("Requesting Logging variables...");
      m_cf.requestLogToc();

      std::function<void(uint32_t, logStablizer*)> cb_stab = std::bind(&CrazyflieROS::onAttitude, this, std::placeholders::_1,std::placeholders::_2);

      logblockStablizer.reset(new LogBlock<logStablizer>(
          &m_cf,{
            {"stabilizer", "roll"},
            {"stabilizer", "pitch"},
            {"stabilizer", "yaw"},
            {"stabilizer", "thrust"},
          }, cb_stab));
        logblockStablizer->start(1); // 10ms


      if (m_enable_logging_imu) {
        std::function<void(uint32_t, logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockImu.reset(new LogBlock<logImu>(
          &m_cf,{
            {"acc", "x"},
            {"acc", "y"},
            {"acc", "z"},
            {"gyro", "x"},
            {"gyro", "y"},
            {"gyro", "z"},
          }, cb));
        logBlockImu->start(1); // 10ms
        //printf("%f\n", cb.acc_x);
        
      }

      if (   m_enable_logging_temperature
          || m_enable_logging_magnetic_field
          || m_enable_logging_pressure
          || m_enable_logging_battery)
      {
        std::function<void(uint32_t, log2*)> cb2 = std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock2.reset(new LogBlock<log2>(
          &m_cf,{
            {"mag", "x"},
            {"mag", "y"},
            {"mag", "z"},
            {"baro", "temp"},
            {"baro", "pressure"},
            {"pm", "vbat"},
          }, cb2));
        logBlock2->start(10); // 100ms
      }

      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }
    }

    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint) {
        m_cf.sendPing();
      }
      m_sentSetpoint = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }



  }

  void onImuData(uint32_t time_in_ms, logImu* data) {

    if (m_enable_logging_imu) {
      sensor_msgs::Imu msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      msg.orientation_covariance[0] = -1;

      // measured in deg/s; need to convert to rad/s
      msg.angular_velocity.x = degToRad(data->gyro_x);
      msg.angular_velocity.y = degToRad(data->gyro_y);
      msg.angular_velocity.z = degToRad(data->gyro_z);

      m_gyro(0) = degToRad(data->gyro_x);
      m_gyro(1) = degToRad(data->gyro_y);
      m_gyro(2) = degToRad(data->gyro_z);

      // measured in mG; need to convert to m/s^2
      msg.linear_acceleration.x = data->acc_x * -9.81;
      msg.linear_acceleration.y = data->acc_y * -9.81;
      msg.linear_acceleration.z = data->acc_z * -9.81;

      m_acc(0) = data->acc_x * -9.81;
      m_acc(1) = data->acc_y * -9.81;
      m_acc(2) = data->acc_z * -9.81; //tested:good

      m_pubImu.publish(msg);
      if(m_enable_logging_magnetic_field && m_mag(2)!=0){
        if(m_init_R)
        {
          m_init_R = false;
          m_prevsT_integ_err = ros::Time::now();
          //printf("MAG_RAW: %f   %f   %f \n", m_mag(0),m_mag(1),m_mag(2)); 
          attitude_reset(&m_acc, &m_mag, &m_att);
        }
        if(!m_init_R){
          ros::Time rn = ros::Time::now();
          float dt = rn.toSec() - m_prevsT_integ_err.toSec();
          attitude_estimation(&m_att, &m_acc, &m_gyro, &m_mag, dt);
          msg_yaw_est.att_est.x = (m_att.Euler)(0);
          msg_yaw_est.att_est.y = (m_att.Euler)(1);
          msg_yaw_est.att_est.z = (m_att.Euler)(2);

          msg_yaw_est.gyro_est.x = m_gyro(0);
          msg_yaw_est.gyro_est.y = m_gyro(1);
          msg_yaw_est.gyro_est.z = m_gyro(2);

          
          m_yawpub.publish(msg_yaw_est);
          m_prevsT_integ_err = ros::Time::now();
          }
      }

    }
  }

  void onAttitude(uint32_t time_in_ms, logStablizer* data)
  {
      m_attest(0) = degToRad(data->roll);
      m_attest(1) = degToRad(data->pitch);
      m_attest(2) = degToRad(data->yaw);
      //printf("Stabilizer Data!!:  %f    %f    %f    %d\n",m_attest(0), m_attest(1), m_attest(2), data->thrust );
  }

  void onLog2Data(uint32_t time_in_ms, log2* data) {
    if (m_enable_logging_temperature) {
      sensor_msgs::Temperature msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      // measured in degC
      msg.temperature = data->baro_temp;
      m_pubTemp.publish(msg);
    }

    if (m_enable_logging_magnetic_field) {
      sensor_msgs::MagneticField msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";

      // measured in Tesla
      m_mag(0) = -data->mag_x;
      m_mag(1) = data->mag_y;
      m_mag(2) = -data->mag_z; //tested: good

      msg.magnetic_field.x = data->mag_x;
      msg.magnetic_field.y = data->mag_y;
      msg.magnetic_field.z = data->mag_z;
      m_pubMag.publish(msg);
      //printf("MAG:  %f    %f    %f\n",m_mag(0),m_mag(1),m_mag(2));
      
      msg_yaw_est.att_est.x = (m_att.Euler)(0);
      msg_yaw_est.att_est.y = (m_att.Euler)(1);
      msg_yaw_est.att_est.z = (m_att.Euler)(2);

      msg_yaw_est.gyro_est.x = m_gyro(0);
      msg_yaw_est.gyro_est.y = m_gyro(1);
      msg_yaw_est.gyro_est.z = m_gyro(2);
      
    }

    if (m_enable_logging_pressure) {
      std_msgs::Float32 msg;
      // hPa (=mbar)
      msg.data = data->baro_pressure;
      m_pubPressure.publish(msg);
    }

    if (m_enable_logging_battery) {
      std_msgs::Float32 msg;
      // V
      msg.data = data->pm_vbat;
      m_pubBattery.publish(msg);
    }
  }
  
  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
    msg.values = *values;

    pub->publish(msg);
  }

  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi.publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("Link Quality low (%f)", linkQuality);
      }
  }


private:
  Crazyflie m_cf;
  std::string m_uri;
  std::string m_tf_prefix;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;
  int m_group_index;
  float m_xh;
  float m_yh;
  bool m_enableLogging;
  bool m_enableParameters;
  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  bool m_use_ros_time;
  bool m_enable_logging_imu;
  bool m_enable_logging_temperature;
  bool m_enable_logging_magnetic_field;
  bool m_enable_logging_pressure;
  bool m_enable_logging_battery;
  bool m_init_R;

  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  ros::Subscriber m_subscribeCmdVel;
  ros::Subscriber m_outputsub;
  ros::Publisher m_pubImu;
  ros::Publisher m_pubTemp;
  ros::Publisher m_pubMag;
  ros::Publisher m_pubPressure;
  ros::Publisher m_pubBattery;
  ros::Publisher m_pubRssi;
  ros::Publisher m_yawpub;
  ros::Time m_prevsT_integ_err;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  crazyflie_driver::Att_est msg_yaw_est;
  bool m_sentSetpoint;
  easyfly::output m_output;

  //Vector4f m_Kcoefs;
  //Vector3f m_init_North;
  //MatrixXf m_erroM;
  //MatrixXf m_erroM;
  //Vector4f m_Kcoefs;
  Att m_att;
  //Attitude_estimator m_Attestimator;
  Vector3f m_acc, m_gyro, m_mag, m_attest;//, m_init_North;

};
//.c_str()
bool add_crazyflie(
  crazyflie_driver::AddCrazyflie::Request  &req,
  crazyflie_driver::AddCrazyflie::Response &res)
{
  	ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d, group_index: %d, g_vehicle_num: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_parameters,
    req.enable_logging,
    req.use_ros_time,
    req.group_index,
    req.g_vehicle_num);

  // Leak intentionally
    CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.group_index,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging,
    req.enable_parameters,
    req.log_blocks,
    req.use_ros_time,
    req.enable_logging_imu,
    req.enable_logging_temperature,
    req.enable_logging_magnetic_field,
    req.enable_logging_pressure,
    req.enable_logging_battery);
  	return true;
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "/vehicle0");
  ros::NodeHandle n;
  char servicename[50];
  //use the absolute addresse for all cfs
  ros::ServiceServer service = n.advertiseService("/add_crazyflie", add_crazyflie);
  ros::spin();

  return 0;
}
