// Lidar Distortion Correction for spherical robots

#include "sr_lidar_distortion_correction.h"

using ::ros::NodeHandle;
using namespace std;


SRLidarDistortionCorrection::SRLidarDistortionCorrection()
        : node_initialized_(false),
          system_ready_(false),
          spin_rate_(1000.0)
{
}


void SRLidarDistortionCorrection::run()
{
    if (!node_initialized_)
    {
        ROS_FATAL("SRLidarDistortionCorrection is not initialized. Shutdown.");
        return;
    }
    ros::spin();
}

// Initialize node
bool SRLidarDistortionCorrection::init() {

  if ( readConfig())
  {
    node_initialized_ = true;
  }
  // create publishers
  pcd_undistorted_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("points_out", 1);
  pcd_distorted_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("points_out2", 1);

  // create subscribers
  pcd_in_sub_ = node_handle_.subscribe("points_in", 1,
      &SRLidarDistortionCorrection::pcdCallback,
      this, ros::TransportHints().tcpNoDelay(true));
  imu_in_sub_ = node_handle_.subscribe("imu_in", 1,
      &SRLidarDistortionCorrection::vehicleTwistCallback,
      this, ros::TransportHints().tcpNoDelay(true));
  
  timer_ = node_handle_.createTimer(ros::Duration(1.0 / spin_rate_),
                                        &SRLidarDistortionCorrection::process, this);

  return node_initialized_;
}


bool SRLidarDistortionCorrection::readConfig()
{
    ros::NodeHandle priv_nh("~");

    // Config arguments
    priv_nh.param<bool>("timing_correction", timing_correction_, false);
    priv_nh.param<int>("spin_rate", spin_rate_, 0);
    priv_nh.param<double>("sr_radius", sr_radius_, 0);

    return true;
}


void SRLidarDistortionCorrection::process(const ros::TimerEvent &)
{
  ros::Time current_time = ros::Time::now();
}


std::vector<double> SRLidarDistortionCorrection::return_twist()
{
  // function for returning the mean rotation speed during frame
  std::vector<double> ret(3);
  ret.at(0) = imu_x / imu_count;
  ret.at(1) = imu_y / imu_count;
  ret.at(2) = imu_z / imu_count;
  imu_count = 0;
  imu_x = 0;
  imu_y = 0;
  imu_z = 0;
  return ret;
}


void SRLidarDistortionCorrection::pcdCallback(const livox_ros_driver::CustomMsg &pcd_in)
{
  // use custom livox format
  pcd_in_ = pcd_in;

  // linear velocity is irrelevant on a spherical robot
  // calculate vehicle's angular velocity, also keep the previous value
  
  //overall rotation speed during frame: averaged -> this is before frame!!
  std::vector<double> rot = return_twist();
  double theta_x = rot.at(0);
  double theta_y = rot.at(1);
  double theta_z = rot.at(2);

  pcl::PointCloud<pcl::PointXYZI> pcl_out;
  pcl::PointCloud<pcl::PointXYZI> pcl_out2;

  auto dt_end = pcd_in_.points[pcd_in_.point_num-1].offset_time * 1e-9;
  double scan_x, scan_y, scan_z;
  for (unsigned int i = 0; i < pcd_in_.point_num; ++i) {
    // offset time
    auto dt = pcd_in_.points[i].offset_time * 1e-9; // from nanoseconds to seconds
    auto dt_ = dt_end - dt;
    //read point coordinates
    scan_x = pcd_in_.points[i].x;
    scan_y = pcd_in_.points[i].y;
    scan_z = pcd_in_.points[i].z;
    if(std::isnan(scan_x) || std::isnan(scan_y) || std::isnan(scan_z) ){continue;} // check for validity
    //perform rotation --> add rottion * dt
    double x_rot = -dt_ * theta_x;
    double y_rot = -dt_ * theta_y;
    double z_rot = -dt_ * theta_z;
    pcl::PointXYZI pt;
    pcl::PointXYZI pt2;
    //write corrected values in pc2. Use rotation rpy-rotation matrix
    pt.x =  scan_x*cos(y_rot)*cos(z_rot) +
            scan_y*(sin(x_rot)*sin(y_rot)*cos(z_rot)-cos(x_rot)*sin(z_rot)) +
            scan_z*(cos(x_rot)*sin(y_rot)*cos(z_rot)+sin(x_rot)*sin(z_rot));
    pt.y =  scan_x*cos(y_rot)*sin(z_rot) +
            scan_y*(sin(x_rot)*sin(y_rot)*sin(z_rot)+cos(x_rot)*cos(z_rot)) +
            scan_z*(cos(x_rot)*sin(y_rot)*sin(z_rot)-sin(x_rot)*cos(z_rot));
    pt.z = -scan_x*sin(y_rot) +
            scan_y*sin(x_rot)*cos(y_rot) +
            scan_z*cos(x_rot)*cos(y_rot);
    if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ){continue;} // check for validity
    pt.intensity = pcd_in_.points[i].reflectivity;
    pcl_out.push_back(pt);
    // for distorted points
    pt2.x = scan_x;
    pt2.y = scan_y;
    pt2.z = scan_z;
    if(std::isnan(pt2.x) || std::isnan(pt2.y) || std::isnan(pt2.z) ){continue;} // check for validity
    pt2.intensity = pt.intensity;
    pcl_out2.push_back(pt2);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_out));
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scan_ptr, msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr2(new pcl::PointCloud<pcl::PointXYZI>(pcl_out2));
  sensor_msgs::PointCloud2 msg2;
  pcl::toROSMsg(*scan_ptr2, msg2);

  msg.header = pcd_in_.header;
  // timing correction
  if(timing_correction_){
    msg.header.stamp = msg.header.stamp + ros::Duration(dt_end);
    msg2.header.stamp = msg.header.stamp + ros::Duration(dt_end);
  }else{
    msg.header.stamp = msg.header.stamp;
    msg2.header.stamp = msg.header.stamp;
  }
  
  // publish
  pcd_undistorted_pub_.publish(msg);
  pcd_distorted_pub_.publish(msg2);
}


void SRLidarDistortionCorrection::vehicleTwistCallback(const sensor_msgs::Imu &imu)
{
  // whenever we read the twist restart process of averaging
  // needs to be alligned to lidar!! 
  // HERE specific for our spherical robot -> axes of LiDAR and IMU need to be alligned
  imu_x += imu.angular_velocity.y;
  imu_y += imu.angular_velocity.x;
  imu_z += - imu.angular_velocity.z;
  imu_count += 1;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "SR_lidar_distortion_correction_node");

  SRLidarDistortionCorrection sr_lidar_distortion_correction_node;
  if (sr_lidar_distortion_correction_node.init()) {
    sr_lidar_distortion_correction_node.run(); 
  }
  else {
    ROS_FATAL_STREAM("SR_lidar_distortion_correction_node initialization failed. Shutdown.");
  }

  return 0;
}
