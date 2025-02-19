#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sam_msgs/msg/topics.hpp>
#include <smarc_msgs/msg/topics.hpp>
#include <smarc_msgs/msg/dvl.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

// GTSAM and GeographicLib includes
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/dataset.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <boost/optional.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <vector>

// Include the GtsamGraph class
#include "hydrobatic_localization/gtsam_graph.h"

using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::B2;

class StateEstimator : public rclcpp::Node {
public:
  StateEstimator();

private:
  // ROS callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void sbg_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg);
  void barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void KeyframeTimerCallback();

  // ROS publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr stim_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sbg_imu_sub_;
  rclcpp::Subscription<smarc_msgs::msg::DVL>::SharedPtr dvl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr est_odometry_pub_;
  rclcpp::Publisher<smarc_msgs::msg::DVL>::SharedPtr dvl_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub_;

  // TF components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcast_;
  geometry_msgs::msg::TransformStamped transformStamped;

  rclcpp::TimerBase::SharedPtr KeyframeTimer;

  // Preintegrated IMU measurements (GTSAM)
  std::shared_ptr<PreintegratedCombinedMeasurements> imu_preintegrated_;
  //Preintegrated SBG measurements (GTSAM)
  std::shared_ptr<PreintegratedCombinedMeasurements> sbg_preintegrated_;

  // Instance of the GtsamGraph class
  GtsamGraph gtsam_graph_;

  // State variables
  int number_of_imu_measurements;
  bool is_graph_initialized_;
  double current_time;
  double last_time_;

  // For initialization
  std::vector<Rot3> estimated_rotations_;
  Point3 initial_position;
  Rot3 initial_rotation;

  // For IMU integration
  Vector3 gyro;

  // DVL
  Vector3 latest_dvl_measurement_;
  bool new_dvl_measurement_;

  // GPS
  Point3 latest_gps_point_;
  bool new_gps_measurement_;
  bool converter_initialized_;
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;

  // Barometer
  double first_barometer_measurement_;
  bool new_barometer_measurement_received_;
  double atmospheric_pressure_;
  double latest_depth_measurement_;

  // Previous state and bias
  NavState previous_state_;
  imuBias::ConstantBias current_imu_bias_;
  imuBias::ConstantBias current_sbg_bias_;

  // Helper functions
  Rot3 averageRotations(const std::vector<Rot3>& rotations);
};

#endif // STATEESTIMATOR_H
