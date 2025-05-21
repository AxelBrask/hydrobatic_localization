#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H
// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sam_msgs/msg/topics.hpp>
#include <smarc_msgs/msg/topics.hpp>
#include <smarc_msgs/msg/dvl.hpp>
#include <smarc_msgs/msg/thruster_feedback.hpp>
#include <smarc_msgs/msg/percent_stamped.hpp>
#include <dead_reckoning_msgs/msg/topics.hpp>
#include <sam_msgs/msg/thruster_angles.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <piml_msgs/msg/thruster_rpm_stamped.hpp> 
#include <sam_msgs/msg/thruster_rp_ms.hpp>
// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// ROS Multithreading
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// GTSAM GeographicLib includes
#include <boost/optional.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <fstream>
#include <chrono>
#include <filesystem> 
// Include the GtsamGraph class
#include "hydrobatic_localization/gtsam_graph.h"
#include "hydrobatic_localization/SamMotionModel.h"

using namespace gtsam;




class StateEstimator : public rclcpp::Node {
public:
  StateEstimator();
  // ~StateEstimator();

private:
  /**
   * @brief Callback function for the IMU sensor, sets the gyro field and integrates the measurements
   * @param msg: the IMU message
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Callback function for the SBG sensor, integrates the measurements
   * @param msg: the SBG message
   */
  void sbg_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Callback function for the DVL sensor, sets the latest DVL measurement
   * @param msg: the DVL message
   */
  void dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg);

  /**
   * @brief Callback function for the barometer sensor, sets the latest barometer measurement
   * @param msg: the barometer message
   */
  void barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

  /**
   * @brief Callback function for the GPS sensor, sets the latest GPS measurement as the relative measuremnt of the navigation frame (odom)
   * @param msg: the GPS message
   */
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  /**
   * @brief Callback for adding thruster vector command to the control sequence queue.
   * @param msg: thruster vector command message
   */
  void ThrusterVectorCallback(const sam_msgs::msg::ThrusterAngles::SharedPtr msg);

  void KeyframeTimerCallback();

  /**
   * @brief Callback for adding thruster RPM command to the control sequence queue.
   * @param msg: thruster RPM command message
   */
  void thruster_callback(const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t1,
                          const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t2);

  /**
   * @brief Callback for adding LCG/VBS command to the control sequence queue.
   * @param msg: LCG/VBS command message
   */
  void lcg_vbs_callback(
    const smarc_msgs::msg::PercentStamped::ConstSharedPtr lcg,
    const smarc_msgs::msg::PercentStamped::ConstSharedPtr vbs);

  void gt_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);


  // ROS publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr stim_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sbg_imu_sub_;
  rclcpp::Subscription<smarc_msgs::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sam_msgs::msg::ThrusterAngles>::SharedPtr thruster_vector_sub_;
  rclcpp::Subscription<sam_msgs::msg::ThrusterRPMs>::SharedPtr thruster_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr motion_model_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_pose_sub_;  

  // ROS Control subscribers with message filters
  // Thruster-only sync
  typedef message_filters::sync_policies::ApproximateTime<piml_msgs::msg::ThrusterRPMStamped,
  piml_msgs::msg::ThrusterRPMStamped> ThrusterSyncPolicy;
  typedef message_filters::Synchronizer<ThrusterSyncPolicy> ThrusterSync;
  std::shared_ptr<ThrusterSync> thruster_sync_;

  // LCG/VBS-only sync
  typedef message_filters::sync_policies::ApproximateTime<smarc_msgs::msg::PercentStamped,
  smarc_msgs::msg::PercentStamped> LcgVbsSyncPolicy;
  typedef message_filters::Synchronizer<LcgVbsSyncPolicy> LcgVbsSync;
  std::shared_ptr<LcgVbsSync> lcg_vbs_sync_;

  // All four subscribers
  message_filters::Subscriber<piml_msgs::msg::ThrusterRPMStamped> thruster1_sub_;
  message_filters::Subscriber<piml_msgs::msg::ThrusterRPMStamped> thruster2_sub_;
  message_filters::Subscriber<smarc_msgs::msg::PercentStamped>     lcg_sub_;
  message_filters::Subscriber<smarc_msgs::msg::PercentStamped>     vbs_sub_;

  std::string config_file_;

  // TF components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcast_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  geometry_msgs::msg::TransformStamped transformStamped;
  std::string name_space_;
  bool init_from_ground_truth_;
  gtsam::Quaternion gt_init_quat_;
  rclcpp::TimerBase::SharedPtr KeyframeTimer;

  // IMU and SBG callback groups
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sbg_callback_group_;

  // Instance of the GtsamGraph class
  std::string inference_strategy_;
  std::unique_ptr<GtsamGraph> gtsam_graph_;

  // State variables
  int number_of_imu_measurements;
  bool is_graph_initialized_;
  // Time variables used for integration
  double current_time;
  double last_time_;

  // For initialization
  std::vector<Rot3> estimated_rotations_;
  Point3 initial_position;
  Rot3 initial_rotation;

  // For IMU integration
  Vector3 gyro; // used for the dvl factor and motion model
  Vector3 dvl_gyro;

  // DVL
  Vector3 latest_dvl_measurement_;
  bool new_dvl_measurement_;

  // GPS
  Point3 latest_gps_point_;
  bool new_gps_measurement_;
  bool map_initialized_;
  double first_utm_x, first_utm_y, first_utm_z;
  int number_of_gps_measurements_;
  int number_of_gps_measurements_for_map_init_ = 5;
  double sum_lat_, sum_lon_, sum_alt_;
  double cov_threshold_ =  10.0;

  // Barometer
  double first_barometer_measurement_;
  bool new_barometer_measurement_received_;
  double atmospheric_pressure_;
  double latest_depth_measurement_;
  double static_offset_;
  bool baro_calibrated = false;

  // Previous state and bias
  NavState previous_state_;
  imuBias::ConstantBias current_imu_bias_;
  imuBias::ConstantBias current_sbg_bias_;


  //Motion model for SAM
  std::shared_ptr<PreintegratedMotionModel> pmm;
  double dt_;
  bool using_motion_model_;
  double last_lcg_{0.0};
  double last_vbs_{0.0};
  double last_thr1_rpm_{0.0};
  double last_thr2_rpm_{0.0};




  // Helper functions
  Rot3 averageRotations(const std::vector<Rot3>& rotations);
};

#endif // STATEESTIMATOR_H
