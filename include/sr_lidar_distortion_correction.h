#ifndef _SR_LIDAR_DISTORTION_CORRECTION_H_
#define _SR_LIDAR_DISTORTION_CORRECTION_H_

#include <cmath>
#include <vector>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <string>


#include <ros/ros.h>
// for Livox data only and imu
#include "livox_ros_driver/CustomMsg.h"
#include <sensor_msgs/Imu.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>





class SRLidarDistortionCorrection {
  
  // Attributes
  bool system_ready_;
  bool node_initialized_;
  double rate_;

  bool timing_correction_ = false;
  int spin_rate_ = 1000;
  double sr_radius_ = 0.15; // in cm

  // for imu
  int imu_count = 0;
  double imu_x = 0.0;
  double imu_y = 0.0;
  double imu_z = 0.0;

  livox_ros_driver::CustomMsg pcd_in_;

  // Node handle
  ros::NodeHandle node_handle_;
  ros::Timer timer_;

  // Published topics
  ros::Publisher pcd_undistorted_pub_;
  ros::Publisher pcd_distorted_pub_;

  // Subsribed topics
  ros::Subscriber pcd_in_sub_;
  ros::Subscriber imu_in_sub_;


  // Methods
  bool readConfig();

  void process(const ros::TimerEvent &);

  std::vector<double> return_twist();

  // Callbacks
  void pcdCallback(const livox_ros_driver::CustomMsg &pcd_in);

  void vehicleTwistCallback(const sensor_msgs::Imu &imu);

  
public:
    SRLidarDistortionCorrection();

    bool init();

    void run();

};

#endif // _SR_LIDAR_DISTORTION_CORRECTION_H_
