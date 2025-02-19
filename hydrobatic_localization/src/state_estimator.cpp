#include "hydrobatic_localization/state_estimator.h"

StateEstimator::StateEstimator()
  : Node("state_estimator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcast_(this),
    number_of_imu_measurements(0),
    is_graph_initialized_(false),
    new_dvl_measurement_(false),
    new_gps_measurement_(false),
    converter_initialized_(false),
    first_barometer_measurement_(0.0),
    new_barometer_measurement_received_(false),
    atmospheric_pressure_(101325.0)
{
  // Subscriptions
  stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::STIM_IMU_TOPIC, 10,
      std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));

  sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::SBG_IMU_TOPIC, 10,
      std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));

  dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
      sam_msgs::msg::Topics::DVL_TOPIC, 10,
      std::bind(&StateEstimator::dvl_callback, this, std::placeholders::_1));

  barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      sam_msgs::msg::Topics::DEPTH_TOPIC, 10,
      std::bind(&StateEstimator::barometer_callback, this, std::placeholders::_1));

  // gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
  //     smarc_msgs::msg::Topics::GPS_TOPIC, 10,
  //     std::bind(&StateEstimator::gps_callback, this, std::placeholders::_1));

  // Use simulation time.
  this->set_parameter(rclcpp::Parameter("use_sim_time", true));

  // Timer for keyframe updates.
  KeyframeTimer = this->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&StateEstimator::KeyframeTimerCallback, this));

  // Publishers
  est_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("estimated_odometry", 10);
  dvl_pub_ = this->create_publisher<smarc_msgs::msg::DVL>("fixed_dvl", 10);
  imu_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("imu_vel_estimate", 10);

  // Initialize the IMU preintegrator using the parameters from the GTSAM side.
  imuBias::ConstantBias prior_bias;
  auto imu_param = gtsam_graph_.getImuParams();
  imu_preintegrated_ = std::make_shared<PreintegratedCombinedMeasurements>(imu_param, prior_bias);

  // Initialize the SBG preintegrator using the parameters from the GTSAM side.
  imuBias::ConstantBias prior_sbg_bias;
  auto sbg_param = gtsam_graph_.getSbgParams();
  sbg_preintegrated_ = std::make_shared<PreintegratedCombinedMeasurements>(sbg_param, prior_sbg_bias);
}

void StateEstimator::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!is_graph_initialized_) {
    number_of_imu_measurements++;
    Rot3 imu_rotation = Rot3::Quaternion(msg->orientation.w,
                                           msg->orientation.x,
                                           msg->orientation.y,
                                           msg->orientation.z);
    estimated_rotations_.push_back(imu_rotation);
    last_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    return;
  }
  current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  number_of_imu_measurements++;

  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  // Adjust the gyro measurements as in the original code.
  gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z());

  double delta_t = 1.0 / 100.0; // or use: current_time - last_time_;
  imu_preintegrated_->integrateMeasurement(acc, gyro, delta_t);
  last_time_ = current_time;
}

void StateEstimator::sbg_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    if (!is_graph_initialized_) {
    // number_of_imu_measurements++;
    // Rot3 imu_rotation = Rot3::Quaternion(msg->orientation.w,
    //                                        msg->orientation.x,
    //                                        msg->orientation.y,
    //                                        msg->orientation.z);
    // estimated_rotations_.push_back(imu_rotation);
    // last_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    return;
  }

  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  // Adjust the gyro measurements as in the original code.
  gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z());

  double delta_t = 1.0 / 100.0; // or use: current_time - last_time_;
  sbg_preintegrated_->integrateMeasurement(acc, gyro, delta_t);
}


void StateEstimator::dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg) {
  Vector3 vel_dvl(msg->velocity.x, msg->velocity.y, msg->velocity.z);
  try {
    geometry_msgs::msg::TransformStamped dvl_to_baselink_tf =
        tf_buffer_.lookupTransform("sam_auv_v1/base_link_gt", "sam_auv_v1/dvl_link_gt", tf2::TimePointZero);

    Rot3 dvl_to_baselink_rot(
        dvl_to_baselink_tf.transform.rotation.w,
        dvl_to_baselink_tf.transform.rotation.x,
        dvl_to_baselink_tf.transform.rotation.y,
        dvl_to_baselink_tf.transform.rotation.z
    );

    Vector3 r_dvl(
        dvl_to_baselink_tf.transform.translation.x,
        dvl_to_baselink_tf.transform.translation.y,
        dvl_to_baselink_tf.transform.translation.z
    );

    Vector3 vel_offset = gyro.cross(r_dvl);
    Vector3 vel_baselink = dvl_to_baselink_rot * vel_dvl - vel_offset;

    latest_dvl_measurement_ = vel_baselink;
    new_dvl_measurement_ = true;

    // Publish the transformed DVL velocity.
    smarc_msgs::msg::DVL transformed_dvl_msg;
    transformed_dvl_msg.header.stamp = this->get_clock()->now();
    transformed_dvl_msg.velocity.x = vel_baselink.x();
    transformed_dvl_msg.velocity.y = vel_baselink.y();
    transformed_dvl_msg.velocity.z = vel_baselink.z();
    dvl_pub_->publish(transformed_dvl_msg);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform DVL velocity: %s", ex.what());
  }
}

void StateEstimator::barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  Vector3 base_to_pressure_offset(-0.503, 0.025, 0.057);
  Rot3 R = previous_state_.pose().rotation();
  Vector3 rotated_offset = R.rotate(base_to_pressure_offset);
  if (!is_graph_initialized_ || first_barometer_measurement_ == 0.0) {
    double measured_pressure = msg->fluid_pressure;
    first_barometer_measurement_ = (measured_pressure- atmospheric_pressure_) / 9806.65 - rotated_offset.z();

    RCLCPP_INFO(this->get_logger(), "First depth measurement: %f", first_barometer_measurement_);
    return;
  }
  double measured_pressure = msg->fluid_pressure;
  double depth = (measured_pressure - atmospheric_pressure_) / 9806.65;

  latest_depth_measurement_ = -depth - rotated_offset.z()- first_barometer_measurement_;
  
  new_barometer_measurement_received_ = true;
}

void StateEstimator::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if (!converter_initialized_) {
    GeographicLib::LocalCartesian temp_cart(msg->latitude, msg->longitude, msg->altitude);
    double lat_bl, lon_bl, alt_bl;
    temp_cart.Reverse(-0.528, 0.0, -0.071, lat_bl, lon_bl, alt_bl);
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(lat_bl, lon_bl, alt_bl);
    converter_initialized_ = true;
  }
  double x, y, z;
  local_cartesian_->Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
  latest_gps_point_ = Point3(x, y, z);
  new_gps_measurement_ = true;
}

Rot3 StateEstimator::averageRotations(const std::vector<Rot3>& rotations) {
  Vector3 sumLog = Vector3::Zero();
  for (const auto& rot : rotations) {
    sumLog += Rot3::Logmap(rot);
  }
  Vector3 avgLog = sumLog / static_cast<double>(rotations.size());
  return Rot3::Expmap(avgLog);
}

void StateEstimator::KeyframeTimerCallback() {
  if (!is_graph_initialized_) {
      // Rot3 inital_orientation = averageRotations(estimated_rotations_);
      try {
        transformStamped = tf_buffer_.lookupTransform("map_gt", "sam_auv_v1/base_link_gt",
                                                        tf2::TimePointZero, std::chrono::seconds(1));
        initial_position = Point3(transformStamped.transform.translation.x,
                                  transformStamped.transform.translation.y,
                                  transformStamped.transform.translation.z);
        initial_rotation = Rot3(transformStamped.transform.rotation.w,
                                transformStamped.transform.rotation.x,
                                transformStamped.transform.rotation.y,
                                transformStamped.transform.rotation.z);
        RCLCPP_INFO(this->get_logger(), "Initial Position: [%f, %f, %f]",
                    initial_position.x(), initial_position.y(), initial_position.z());
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
      }

      // Broadcast the initial pose.
      geometry_msgs::msg::TransformStamped init_transform;
      init_transform.header.stamp = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "Time: %f",
                  init_transform.header.stamp.sec + init_transform.header.stamp.nanosec * 1e-9);
      init_transform.header.frame_id = "sam_auv_v1/odom_gt";
      init_transform.child_frame_id = "estimated_pose";
      init_transform.transform.translation.x = initial_position.x();
      init_transform.transform.translation.y = initial_position.y();
      init_transform.transform.translation.z = initial_position.z();
      Quaternion quat = initial_rotation.toQuaternion();
      init_transform.transform.rotation.x = quat.x();
      init_transform.transform.rotation.y = quat.y();
      init_transform.transform.rotation.z = quat.z();
      init_transform.transform.rotation.w = quat.w();
      tf_broadcast_.sendTransform(init_transform);

      // Initialize the GTSAM graph and state.
      gtsam_graph_.initGraphAndState(initial_rotation, initial_position);
      is_graph_initialized_ = true;
      return;
    
  }

  // If IMU is initialized, update the graph with new measurements.
  gtsam_graph_.addImuFactor(*imu_preintegrated_, previous_state_, current_imu_bias_);

  // // Publish the IMU velocity estimate.
  // nav_msgs::msg::Odometry imu_odom_msg;
  // imu_odom_msg.header.stamp = this->get_clock()->now();
  // imu_odom_msg.header.frame_id = "sam_auv_v1/odom_gt";
  // imu_odom_msg.child_frame_id = "sam_auv_v1/base_link_gt";
  // imu_odom_msg.pose.pose.position.x = predicted_state.pose().translation().x();
  // imu_odom_msg.pose.pose.position.y = predicted_state.pose().translation().y();
  // imu_odom_msg.pose.pose.position.z = predicted_state.pose().translation().z();
  // Quaternion quat_imu = predicted_state.pose().rotation().toQuaternion();
  // imu_odom_msg.pose.pose.orientation.x = quat_imu.x();
  // imu_odom_msg.pose.pose.orientation.y = quat_imu.y();
  // imu_odom_msg.pose.pose.orientation.z = quat_imu.z();
  // imu_odom_msg.pose.pose.orientation.w = quat_imu.w();
  // imu_odom_msg.twist.twist.linear.x = predicted_state.v().x();
  // imu_odom_msg.twist.twist.linear.y = predicted_state.v().y();
  // imu_odom_msg.twist.twist.linear.z = predicted_state.v().z();
  // imu_odom_pub->publish(imu_odom_msg);

  NavState predicted_sbg_stat = gtsam_graph_.addSbgFactor(*sbg_preintegrated_, previous_state_, current_sbg_bias_);

  if (new_dvl_measurement_) {
    Vector3 dvl_velocity = previous_state_.rotation() * latest_dvl_measurement_;
    gtsam_graph_.addDvlFactor(dvl_velocity);
    new_dvl_measurement_ = false;
  }

  if (new_gps_measurement_) {
    gtsam_graph_.addGpsFactor(latest_gps_point_);
    new_gps_measurement_ = false;
  }

  if (new_barometer_measurement_received_) {
    RCLCPP_INFO(this->get_logger(), "BarometerFactor Depth: %f", latest_depth_measurement_);
    gtsam_graph_.addBarometerFactor(latest_depth_measurement_);
    new_barometer_measurement_received_ = false;
  }

  // Optimize the factor graph.
  gtsam_graph_.optimize();

  // Update the current state and bias.
  current_imu_bias_ = gtsam_graph_.getCurrentImuBias();
  current_sbg_bias_ = gtsam_graph_.getCurrentSbgBias();
  previous_state_ = gtsam_graph_.getCurrentState();

  // Reset the IMU preintegrator with the updated bias.
  imu_preintegrated_->resetIntegrationAndSetBias(current_imu_bias_);
  sbg_preintegrated_->resetIntegrationAndSetBias(current_sbg_bias_);

  // Publish the estimated odometry.
  nav_msgs::msg::Odometry odometry_msg;
  odometry_msg.header.stamp = this->get_clock()->now();
  odometry_msg.header.frame_id = "sam_auv_v1/odom_gt";
  odometry_msg.child_frame_id = "sam_auv_v1/base_link_gt";
  odometry_msg.pose.pose.position.x = previous_state_.pose().translation().x();
  odometry_msg.pose.pose.position.y = previous_state_.pose().translation().y();
  odometry_msg.pose.pose.position.z = previous_state_.pose().translation().z();
  Quaternion gtsamQuat = previous_state_.pose().rotation().toQuaternion();
  odometry_msg.pose.pose.orientation.x = gtsamQuat.x();
  odometry_msg.pose.pose.orientation.y = gtsamQuat.y();
  odometry_msg.pose.pose.orientation.z = gtsamQuat.z();
  odometry_msg.pose.pose.orientation.w = gtsamQuat.w();
  odometry_msg.twist.twist.linear.x = previous_state_.v().x();
  odometry_msg.twist.twist.linear.y = previous_state_.v().y();
  odometry_msg.twist.twist.linear.z = previous_state_.v().z();
  est_odometry_pub_->publish(odometry_msg);

  // Broadcast estimated pose.
  geometry_msgs::msg::TransformStamped out_transform;
  out_transform.header.stamp = this->get_clock()->now();
  out_transform.header.frame_id = "sam_auv_v1/odom_gt";
  out_transform.child_frame_id = "estimated_pose";
  Point3 estimated_translation = previous_state_.pose().translation();
  Rot3 estimated_rotation = previous_state_.pose().rotation();
  out_transform.transform.translation.x = estimated_translation.x();
  out_transform.transform.translation.y = estimated_translation.y();
  out_transform.transform.translation.z = estimated_translation.z();
  Quaternion out_quat = estimated_rotation.toQuaternion();
  out_transform.transform.rotation.x = out_quat.x();
  out_transform.transform.rotation.y = out_quat.y();
  out_transform.transform.rotation.z = out_quat.z();
  out_transform.transform.rotation.w = out_quat.w();
  tf_broadcast_.sendTransform(out_transform);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
