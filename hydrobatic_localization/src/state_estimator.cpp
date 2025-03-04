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
      sam_msgs::msg::Topics::STIM_IMU_TOPIC, 1000,
      std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));

  sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::SBG_IMU_TOPIC, 1000,
      std::bind(&StateEstimator::sbg_callback, this, std::placeholders::_1));

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
   tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  // Timer for keyframe updates.
  KeyframeTimer = this->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&StateEstimator::KeyframeTimerCallback, this));
  dvl_pub_ = this->create_publisher<smarc_msgs::msg::DVL>("dvl", 10);
  // Initialize the GtsamGraph
  gtsam_graph_ = std::make_unique<GtsamGraph>();
}

void StateEstimator::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z()); // Adjusted gyro measurements to right-hand rule.
  double delta_t = 1.0 / 100.0; // or use: current_time - last_time_;
 
  gtsam_graph_->integrateImuMeasurement(acc, gyro, delta_t);
  last_time_ = current_time;
}

void StateEstimator::sbg_callback(const sensor_msgs::msg::Imu::SharedPtr msg){

  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  Vector3 sbg_gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z());

  double delta_t = 1.0 / 100.0; // or use: current_time - last_time_;
  gtsam_graph_->integrateSbgMeasurement(acc, sbg_gyro, delta_t);
}


void StateEstimator::dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg) {
   Vector3 vel_dvl(msg->velocity.x, msg->velocity.y, msg->velocity.z);

//   //   Vector3 vel_offset = gyro.cross(r_dvl);
// //   Vector3 vel_baselink = dvl_to_baselink_rot * vel_dvl - vel_offset;
//     Vector3 base_link_to_dvl_offset(0.573 ,0.0 ,-0.063); // translation from dvl_link to base_link
//     Rot3 base_link_dvl_rotation = Rot3::Identity();
//     // Vector3 dvl_velocity = previous_state_.rotation() * latest_dvl_measurement_;
//       Rot3 R_world_base_link = previous_state_.rotation();
//     // velocity of the dvl frame without the angular velocity contribution
//     RCLCPP_INFO(this->get_logger(), "Previous state velocity: [%f, %f, %f]", previous_state_.velocity().x(), previous_state_.velocity().y(), previous_state_.velocity().z());
//     Vector3  vel_world_dvl = base_link_dvl_rotation.matrix()*(R_world_base_link.transpose().matrix()*previous_state_.velocity());
//     // angular velocity contribution
//     Vector3 angualr_velocity_contribution = base_link_to_dvl_offset.cross(base_link_dvl_rotation*(gyro));
//     // expected velocity of the dvl frame
//     Vector3 expected_velocity = vel_world_dvl + angualr_velocity_contribution;
//     // pub dvl
//     smarc_msgs::msg::DVL dvl_msg;
//     dvl_msg.header.stamp = this->get_clock()->now();
//     dvl_msg.velocity.x = expected_velocity.x();
//     dvl_msg.velocity.y = expected_velocity.y();
//     dvl_msg.velocity.z = expected_velocity.z();
//     dvl_pub_->publish(dvl_msg);  
    latest_dvl_measurement_ = vel_dvl;
    new_dvl_measurement_ = true;
}


void StateEstimator::barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  double measured_pressure = msg->fluid_pressure;
  double depth = -(measured_pressure - atmospheric_pressure_) / 9806.65; //Down negative

  if (!is_graph_initialized_ || first_barometer_measurement_ == 0.0) {
        try {
        transformStamped = tf_buffer_.lookupTransform("map_gt", "sam_auv_v1/odom_gt",
                                                      tf2::TimePointZero, std::chrono::seconds(1));
        static_offset_ = transformStamped.transform.translation.z;

      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
      }
    first_barometer_measurement_ =  - static_offset_ - depth ;
    return;
  }
  
  latest_depth_measurement_ =  depth - static_offset_; // depth in the odom frame
  // RCLCPP_INFO(this->get_logger(), "Depth: %f", latest_depth_measurement_);
  new_barometer_measurement_received_ = true;
}


void StateEstimator::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if (!converter_initialized_) {
    try {
    // Convert lat/lon to UTM
    int utm_zone;
    bool northp;
    double utm_x, utm_y;
    GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone, northp, utm_x, utm_y);
    double utm_z = msg->altitude;
    // Create a static transform from "utm" to "map" using the UTM coordinates.
    geometry_msgs::msg::TransformStamped map_transform;
    map_transform.header.stamp = this->get_clock()->now();
    map_transform.header.frame_id = "utm";     // Parent frame
    map_transform.child_frame_id = "map";        // New map frame
    map_transform.transform.translation.x = utm_x;
    map_transform.transform.translation.y = utm_y;
    map_transform.transform.translation.z = utm_z;
    // Use an identity rotation for the map frame.
    map_transform.transform.rotation.x = 0.0;
    map_transform.transform.rotation.y = 0.0;
    map_transform.transform.rotation.z = 0.0;
    map_transform.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(map_transform);

      transformStamped = tf_buffer_.lookupTransform("sam_auv_v1/gps_link_gt","sam_auv_v1/odom_gt",
                                                      tf2::TimePointZero, std::chrono::seconds(1));
      gtsam::Point3 gps_to_odom_offset = Point3(transformStamped.transform.translation.x,
                                transformStamped.transform.translation.y,
                                transformStamped.transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "GPS to Odom Offset: [%f, %f, %f]",
                gps_to_odom_offset.x(), gps_to_odom_offset.y(), gps_to_odom_offset.z());

    GeographicLib::LocalCartesian temp_cart(msg->latitude, msg->longitude, msg->altitude);
    double lat_bl, lon_bl, alt_bl;
    temp_cart.Reverse(gps_to_odom_offset.x(), gps_to_odom_offset.y(), gps_to_odom_offset.z(), lat_bl, lon_bl, alt_bl);
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(lat_bl, lon_bl, alt_bl);
    converter_initialized_ = true;

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }

  }
  double x, y, z;
  local_cartesian_->Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
  double x_N = -y;
  double y_W = x;
  double z_U = z;
  RCLCPP_INFO(this->get_logger(), "GPS point in odom_frame [%f, %f, %f]", x_N, y_W, z_U);
  Vector3 base_to_gps_offset(0.528 ,0.0, 0.071);
  latest_gps_point_ = Point3(x_N-base_to_gps_offset.y(),y_W-base_to_gps_offset.x(),z_U-base_to_gps_offset.z()); // change becuase of odom being twisted +90 in z
  RCLCPP_INFO(this->get_logger(), "GPS point: [%f, %f, %f]", latest_gps_point_.x(), latest_gps_point_.y(), latest_gps_point_.z());
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
        transformStamped = tf_buffer_.lookupTransform("sam_auv_v1/odom_gt", "sam_auv_v1/base_link_gt",
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
      gtsam_graph_->initGraphAndState(initial_rotation, initial_position);
      previous_state_ = gtsam_graph_->getCurrentState();
      is_graph_initialized_ = true;
      return;
    
  }

  // If IMU is initialized, update the graph with new measurements.
  NavState predictes_imu_state = gtsam_graph_->addImuFactor(previous_state_, current_imu_bias_);
  // RCLCPP_INFO(this->get_logger(), "Predicted IMU State: [%f, %f, %f]",
  //             predictes_imu_state.pose().translation().x(),
  //             predictes_imu_state.pose().translation().y(),
  //             predictes_imu_state.pose().translation().z());

  NavState predicted_sbg_state = gtsam_graph_->addSbgFactor(previous_state_, current_sbg_bias_);

  if (new_dvl_measurement_) {  
    gtsam_graph_->addDvlFactor(latest_dvl_measurement_, gyro );
    new_dvl_measurement_ = false;
  }

  if (new_gps_measurement_) {
    gtsam_graph_->addGpsFactor(latest_gps_point_);
    new_gps_measurement_ = false;
  }

  if (new_barometer_measurement_received_) {
    gtsam_graph_->addBarometerFactor(latest_depth_measurement_);
    new_barometer_measurement_received_ = false;
  }

  // Optimize the factor graph.
  gtsam_graph_->optimize();

  // Update the current state and bias.
  current_imu_bias_ = gtsam_graph_->getCurrentImuBias();
  current_sbg_bias_ = gtsam_graph_->getCurrentSbgBias();
  previous_state_ = gtsam_graph_->getCurrentState();


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
