#include "ros_api.h"
#include "ldlidar_driver.h"

void ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, 
    LaserScanSetting& setting, ros::Publisher& lidarpub, ros::Publisher& corrected_lidarpub,
    double rotation_angle, double correction_amount);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldldiar_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string product_name;
  std::string topic_name;
  std::string corrected_topic_name;
  std::string port_name;
  int serial_port_baudrate;
  LaserScanSetting setting;
  ldlidar::LDType type_name;

  double rotation_angle;
  double correction_amount;
  nh_private.param("rotation_angle", rotation_angle, 0.0);
  nh_private.param("correction_amount", correction_amount, 0.0);
  nh_private.getParam("product_name", product_name);
  nh_private.getParam("topic_name", topic_name);
  nh_private.getParam("corrected_topic_name", corrected_topic_name);
  nh_private.param("frame_id", setting.frame_id, std::string("base_laser"));
  nh_private.getParam("port_name", port_name);
  nh_private.param("port_baudrate", serial_port_baudrate, int(230400));
  nh_private.param("laser_scan_dir", setting.laser_scan_dir, bool(true));
  nh_private.param("enable_angle_crop_func", setting.enable_angle_crop_func, bool(false));
  nh_private.param("angle_crop_min", setting.angle_crop_min, double(0.0));
  nh_private.param("angle_crop_max", setting.angle_crop_max, double(0.0));

  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  ROS_INFO("LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  ROS_INFO("ROS params input:");
  ROS_INFO("<product_name>: %s", product_name.c_str());
  ROS_INFO("<topic_name>: %s", topic_name.c_str());
  ROS_INFO("<corrected_topic_name>: %s", corrected_topic_name.c_str());
  ROS_INFO("<frame_id>: %s", setting.frame_id.c_str());
  ROS_INFO("<port_name>: %s", port_name.c_str());
  ROS_INFO("<port_baudrate>: %d", serial_port_baudrate);
  ROS_INFO("<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  ROS_INFO("<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  ROS_INFO("<angle_crop_min>: %f", setting.angle_crop_min);
  ROS_INFO("<angle_crop_max>: %f", setting.angle_crop_max);
  ROS_INFO("<rotation_angle>: %f", rotation_angle);
  ROS_INFO("<correction_amount>: %f", correction_amount);

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06; 
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else {
    ROS_ERROR("Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 
  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    ROS_INFO("ldlidar node start is success");
  } else {
    ROS_ERROR("ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (ldlidarnode->WaitLidarCommConnect(3000)) {
    ROS_INFO("ldlidar communication is normal.");
  } else {
    ROS_ERROR("ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 10);
  ros::Publisher corrected_lidar_pub = nh.advertise<sensor_msgs::LaserScan>(corrected_topic_name, 10);

  ros::Rate r(10); //10hz
  ldlidar::Points2D laser_scan_points;
  double lidar_scan_freq;
  ROS_INFO("Publish topic messages: ldlidar scan data and corrected scan data.");

  while (ros::ok()) {
    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL: 
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting, lidar_pub, corrected_lidar_pub, rotation_angle, correction_amount);
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        ROS_ERROR("get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }
    r.sleep();
  }

  ldlidarnode->Stop();
  delete ldlidarnode;
  ldlidarnode = nullptr;

  return 0;
}

void ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, 
    LaserScanSetting& setting, ros::Publisher& lidarpub, ros::Publisher& corrected_lidarpub,
    double rotation_angle, double correction_amount) {

  float angle_min, angle_max, range_min, range_max, angle_increment;
  float scan_time;
  ros::Time start_scan_time;
  static ros::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  angle_min = 0;
  angle_max = (2 * M_PI);
  range_min = 0.02;
  range_max = 12;
  int beam_size = static_cast<int>(src.size());
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);

  if (lidar_spin_freq > 0) {
    sensor_msgs::LaserScan output;
    sensor_msgs::LaserScan corrected_output;

    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    
    corrected_output = output;  // Copy all fields from output

    if (beam_size <= 1) {
      output.time_increment = 0;
      corrected_output.time_increment = 0;
    } else {
      output.time_increment = scan_time / (float)(beam_size - 1);
      corrected_output.time_increment = output.time_increment;
    }
    output.scan_time = scan_time;
    corrected_output.scan_time = scan_time;

    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    corrected_output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    corrected_output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    const float epsilon = 1e-6f; 
    for (auto point : src) {
      float range = point.distance / 1000.f; // Convert distance to meters
      float intensity = point.intensity;
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) {
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      // **A check for zero range values which assigns them to NaN**
      if (std::abs(range) < epsilon) {
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) {
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float rotated_angle = dir_angle + rotation_angle;
      if (rotated_angle >= 360.0) rotated_angle -= 360.0;
      if (rotated_angle < 0.0) rotated_angle += 360.0;

      float angle = ANGLE_TO_RADIAN(rotated_angle);
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
      if (index < beam_size) {
        if (index < 0) {
          ROS_ERROR("[ldrobot] error index: %d, beam_size: %d, angle: %f, angle_min: %f, angle_increment: %f", 
              index, beam_size, angle, angle_min, angle_increment);
        }

        int actual_index = setting.laser_scan_dir ? (beam_size - index - 1) : index;

        if (std::isnan(output.ranges[actual_index]) || (range < output.ranges[actual_index])) {
          output.ranges[actual_index] = range;

          if (std::isnan(range)) {
            corrected_output.ranges[actual_index] = std::numeric_limits<float>::quiet_NaN();
          } else {
            corrected_output.ranges[actual_index] = range + correction_amount;
          }
        }

        output.intensities[actual_index] = intensity;
        corrected_output.intensities[actual_index] = intensity;
      }
    }
    lidarpub.publish(output);
    corrected_lidarpub.publish(corrected_output);
    end_scan_time = start_scan_time;
  } 
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}