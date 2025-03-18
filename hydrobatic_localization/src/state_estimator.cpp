#include "hydrobatic_localization/state_estimator.h"




// TODO: create uniqe lock for keyframe timer, and set the notification when baromter and gps are received
// Parse xacro file, for extrinsics
StateEstimator::StateEstimator()
  : Node("state_estimator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    tf_broadcast_(this), number_of_imu_measurements(0), is_graph_initialized_(false),
    new_dvl_measurement_(false), new_gps_measurement_(false), map_initialized_(false),
    first_barometer_measurement_(0.0), new_barometer_measurement_received_(false), atmospheric_pressure_(101325.0),
    dt_(0.01)
{
  // Subscriptions for sensors
  stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::STIM_IMU_TOPIC, 10,
      std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));

  // sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
  //     sam_msgs::msg::Topics::SBG_IMU_TOPIC, 100,
  //     std::bind(&StateEstimator::sbg_callback, this, std::placeholders::_1));

  dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
      sam_msgs::msg::Topics::DVL_TOPIC, 10,
      std::bind(&StateEstimator::dvl_callback, this, std::placeholders::_1));

  barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      sam_msgs::msg::Topics::DEPTH_TOPIC, 10,
      std::bind(&StateEstimator::barometer_callback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      smarc_msgs::msg::Topics::GPS_TOPIC, 10,
      std::bind(&StateEstimator::gps_callback, this, std::placeholders::_1));

  thruster_vector_sub_ = this-> create_subscription<sam_msgs::msg::ThrusterAngles>(
      sam_msgs::msg::Topics::THRUST_VECTOR_CMD_TOPIC, 10,
      [this](const sam_msgs::msg::ThrusterAngles::SharedPtr msg) {
        latest_thruster_vector_ = *msg;
      }
  );

  // Subscriptions for control inputs with message filters
  thruster1_sub_.subscribe(this, sam_msgs::msg::Topics::THRUSTER1_FB_TOPIC);
  thruster2_sub_.subscribe(this, sam_msgs::msg::Topics::THRUSTER2_FB_TOPIC);
  lcg_sub_.subscribe(this, sam_msgs::msg::Topics::LCG_FB_TOPIC);
  vbs_sub_.subscribe(this, sam_msgs::msg::Topics::VBS_FB_TOPIC);
  // thruster_vector_sub_.subscribe(this, sam_msgs::msg::Topics::THRUST_VECTOR_CMD_TOPIC);

  sync_ = std::make_shared<Sync>(SyncPolicy(10), thruster1_sub_, thruster2_sub_,
                                 lcg_sub_, vbs_sub_);

  sync_->registerCallback(std::bind(&StateEstimator::control_input_callback, this,
                          std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4));


  // Use simulation time.
  this->set_parameter(rclcpp::Parameter("use_sim_time", true));
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);  


  // Timer for keyframe updates. This should be changed to a seperate thread
  KeyframeTimer = this->create_wall_timer(
      std::chrono::milliseconds(400), std::bind(&StateEstimator::KeyframeTimerCallback, this));

  //Initialize the Sam Motion Model
  sam_motion_model_ = std::make_unique<SamMotionModelWrapper>(dt_);
  // Initialize the GtsamGraph
  gtsam_graph_ = std::make_unique<GtsamGraph>();
}


void StateEstimator::control_input_callback(const smarc_msgs::msg::ThrusterFeedback::ConstSharedPtr thruster1,
                                            const smarc_msgs::msg::ThrusterFeedback::ConstSharedPtr thruster2,
                                            const smarc_msgs::msg::PercentStamped::ConstSharedPtr lcg,
                                            const smarc_msgs::msg::PercentStamped::ConstSharedPtr vbs) {

  if(is_graph_initialized_){


    Eigen::VectorXd u(6);
    u << lcg->value, vbs->value, latest_thruster_vector_.thruster_vertical_radians, latest_thruster_vector_.thruster_horizontal_radians, thruster1->rpm.rpm, thruster2->rpm.rpm;
  
    if(new_current_integration_state_){
      // Convert the Navstate to the corrrect state space vector (eta and nu)
      current_integration_state_ = sam_motion_model_->stateToVector(gtsam_graph_->getCurrentState(), gyro);
      RCLCPP_INFO(this->get_logger(), "New integration state set");
      new_current_integration_state_ = false;
    }
  
    if(!initial_control_input_received_){
      initial_control_input_received_ = true;
      sam_motion_model_-> setPrevControl(u);
      return; 
    }
    Eigen::VectorXd x(current_integration_state_.size()+u.size());
    x.head(current_integration_state_.size()) = current_integration_state_;
    x.tail(u.size()) = u;

    for(int i = 0; i < 100; i++){
      x = sam_motion_model_->integrateState(x, u, 0.001); // Inlcudes dynamics and integration
    }
    current_integration_state_ = x.head(current_integration_state_.size());

    // Print out the updated state vector
    Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", ", ");
    std::stringstream ss;
    ss << x.transpose().format(fmt);
    std::string s2 = ss.str();
    RCLCPP_INFO(this->get_logger(), "x: %s", s2.c_str());


  }


}



void StateEstimator::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z()); // Adjusted gyro measurements to right-hand rule.

  // Current initialization of the graph is based on the first 6 imu measurements, 
  if(number_of_imu_measurements< 6){
    Rot3 current_rotation = Rot3::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    estimated_rotations_.push_back(current_rotation);
    number_of_imu_measurements++;
  }

  double delta_t = 1.0 / 100.0;
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
    latest_dvl_measurement_ = vel_dvl;
    dvl_gyro = gyro;
    new_dvl_measurement_ = true;
}



void StateEstimator::barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  double measured_pressure = msg->fluid_pressure;
  double depth = -(measured_pressure - atmospheric_pressure_) / 9806.65; //Down negative

  if (!is_graph_initialized_ && !map_initialized_) {
        try {
        transformStamped = tf_buffer_.lookupTransform("map", "odom",
                                                      tf2::TimePointZero, std::chrono::seconds(1));
        static_offset_ = transformStamped.transform.translation.z;

      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
      }
    first_barometer_measurement_ =  - static_offset_ - depth ;
    latest_depth_measurement_ =  depth - static_offset_; // depth in the odom frame
    new_barometer_measurement_received_ = true;
    return;
  }
  
  latest_depth_measurement_ =  depth - static_offset_; // depth in the odom frame
  new_barometer_measurement_received_ = true;
}



void StateEstimator::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  double utm_x, utm_y, utm_z;
  // if sim time is used, take the ground truth as gps reading
  if(this->get_parameter("use_sim_time").as_bool()){
    try{
      transformStamped = tf_buffer_.lookupTransform("utm_34_V", "sam_auv_v1/gps_link_gt",
                                                      tf2::TimePointZero, std::chrono::seconds(1));
      utm_x = transformStamped.transform.translation.x;
      utm_y = transformStamped.transform.translation.y;
      utm_z = transformStamped.transform.translation.z;
    }

    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }
  }
  // if not using the sim, take the real gps reading
  else {
    int utm_zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone, northp, utm_x, utm_y);
    utm_z = msg->altitude;

  }

 if (!map_initialized_) {

    // Create a static transform from "utm" to "map" using the UTM coordinates.
    geometry_msgs::msg::TransformStamped map_transform;
    map_transform.header.stamp = this->get_clock()->now();
    map_transform.header.frame_id = "utm_34_V";     // Frmae name from sim
    map_transform.child_frame_id = "map";        
    map_transform.transform.translation.x = utm_x;
    map_transform.transform.translation.y = utm_y;
    map_transform.transform.translation.z = utm_z;
    // Use an identity rotation for the map frame.
    map_transform.transform.rotation.x = 0.0;
    map_transform.transform.rotation.y = 0.0;
    map_transform.transform.rotation.z = 0.0;
    map_transform.transform.rotation.w = 1.0;
    tf_static_broadcaster_->sendTransform(map_transform);
    // first utm coordinates of the base_link
    first_utm_x = utm_x;
    first_utm_y = utm_y;
    first_utm_z = utm_z;
    RCLCPP_INFO(this->get_logger(), 
                "Broadcasted static map transform at local x: %f, y: %f, z: %f", 
                utm_x, utm_y, utm_z);
    map_initialized_ = true;
    return;
 }
  // Compare the new gps message with the first one to get the offset, but we need it in the odom frame
  if(is_graph_initialized_){
    Point3 map_to_odom_offset;
    Rot3 map_to_odom_rotation;
    try{
      transformStamped = tf_buffer_.lookupTransform("map", "odom",
                                                      tf2::TimePointZero, std::chrono::seconds(1));
      map_to_odom_offset = Point3(transformStamped.transform.translation.x,
                                  transformStamped.transform.translation.y,
                                  transformStamped.transform.translation.z);
      map_to_odom_rotation = Rot3(transformStamped.transform.rotation.w,
                                  transformStamped.transform.rotation.x,
                                  transformStamped.transform.rotation.y,
                                  transformStamped.transform.rotation.z);

    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }
    // this assumes that map to odom is only a translation, this should be changed to a full transformation with the help of a compass for heading
    double x = utm_x - first_utm_x - map_to_odom_offset.x();
    double y = utm_y - first_utm_y - map_to_odom_offset.y();
    double z = utm_z - first_utm_z - map_to_odom_offset.z();
    latest_gps_point_ = Point3(x, y, z);
    new_gps_measurement_ = true;
    // Logg off the gps point of the gps in the odom frame
    RCLCPP_DEBUG(this->get_logger(), "GPS Point: [%f, %f, %f]", latest_gps_point_.x(), latest_gps_point_.y(), latest_gps_point_.z());
  }
 
}



Rot3 StateEstimator::averageRotations(const std::vector<Rot3>& rotations) {
  Vector3 sumLog = Vector3::Zero();
  for (const auto& rot : rotations) {
    sumLog += Rot3::Logmap(rot);
  }
  Vector3 avgLog = sumLog / static_cast<double>(rotations.size());
  return Rot3::Expmap(avgLog);
}



void StateEstimator::KeyframeThread() {
}


void StateEstimator::KeyframeTimerCallback(){
  // need to have at least 6 imu measurements to initialize the graph with the current orientation
    if(number_of_imu_measurements < 6){
      return;
      }

    if (!is_graph_initialized_ && map_initialized_) {
        // Initialize the odom frame from map
        Point3 base_to_gps_offset(0.528 ,0.0, 0.071);
        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.header.stamp = this->get_clock()->now();
        odom_transform.header.frame_id = "map";
        odom_transform.child_frame_id = "odom";
        // translate the map -> odom with -base_to_gps_offset in x and y
        odom_transform.transform.translation.x = -base_to_gps_offset.y();
        odom_transform.transform.translation.y = -base_to_gps_offset.x();
        odom_transform.transform.translation.z = -base_to_gps_offset.z();
        odom_transform.transform.rotation.x = 0.0;
        odom_transform.transform.rotation.y = 0.0;
        odom_transform.transform.rotation.z = 0.0;
        odom_transform.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(odom_transform);

        // TODO take the orientation of the compass as the initial orientation
        // start the graph with average orientation from the imu,
        Rot3 average_rotation = averageRotations(estimated_rotations_);
        // For now let the starting position be the origin of odom frame
        Point3 initial_position = Point3(0.0, 0.0, 0.0);
        // Broadcast the initial pose.
        geometry_msgs::msg::TransformStamped init_transform;
        init_transform.header.stamp = this->get_clock()->now();
        init_transform.header.frame_id = "odom";
        init_transform.child_frame_id = "estimated_pose";
        init_transform.transform.translation.x = initial_position.x();
        init_transform.transform.translation.y = initial_position.y();
        init_transform.transform.translation.z = initial_position.z();
        Quaternion quat = average_rotation.toQuaternion();
        init_transform.transform.rotation.x = quat.x();
        init_transform.transform.rotation.y = quat.y();
        init_transform.transform.rotation.z = quat.z();
        init_transform.transform.rotation.w = quat.w();
        tf_broadcast_.sendTransform(init_transform);

        // Initialize the GTSAM graph and state.
        gtsam_graph_->initGraphAndState(average_rotation, initial_position);
        previous_state_ = gtsam_graph_->getCurrentState();
        is_graph_initialized_ = true;
        new_current_integration_state_ = true;
        return;
      
    }


    // Predict the next state using the preintegrated measurements AND add the imu factor to the graph.
    NavState predictes_imu_state = gtsam_graph_->addImuFactor();
    // RCLCPP_INFO(this->get_logger(), "Predicted IMU State: [%f, %f, %f]",
    //             predictes_imu_state.pose().translation().x(),
    //             predictes_imu_state.pose().translation().y(),
    //             predictes_imu_state.pose().translation().z());

    // NavState predicted_sbg_state = gtsam_graph_->addSbgFactor();
    // RCLCPP_INFO(this->get_logger(), "Predicted SBG State: [%f, %f, %f]",
    //             predicted_sbg_state.pose().translation().x(),
    //             predicted_sbg_state.pose().translation().y(),
    //             predicted_sbg_state.pose().translation().z());

    // Add the DVL, GPS and Barometer factors to the graph.
    if (new_dvl_measurement_) {  
      gtsam_graph_->addDvlFactor(latest_dvl_measurement_, dvl_gyro );
      new_dvl_measurement_ = false;
    }

    if (new_gps_measurement_) {
      gtsam_graph_->addGpsFactor(latest_gps_point_);
      new_gps_measurement_ = false;
    }

    if (new_barometer_measurement_received_) {
      gtsam_graph_->addBarometerFactor(latest_depth_measurement_);
      // RCLCPP_INFO(this->get_logger(), "Barometer Factor Added for Depth: %f", latest_depth_measurement_);
      new_barometer_measurement_received_ = false;
    }

    // Optimize the factor graph.
    gtsam_graph_->optimize();
    new_current_integration_state_ = true;
    // Update the current state and bias.
    current_imu_bias_ = gtsam_graph_->getCurrentImuBias();
    // current_sbg_bias_ = gtsam_graph_->getCurrentSbgBias();
    previous_state_ = gtsam_graph_->getCurrentState();
    //log the current state
    RCLCPP_INFO(this->get_logger(), "Current State: [%f, %f, %f]",
                previous_state_.pose().translation().x(),
                previous_state_.pose().translation().y(),
                previous_state_.pose().translation().z());

    // Broadcast estimated pose.
    geometry_msgs::msg::TransformStamped out_transform;
    out_transform.header.stamp = this->get_clock()->now();
    out_transform.header.frame_id = "odom";
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
  py::scoped_interpreter guard{};
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
