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


  // Declare parameters
  this->declare_parameter<bool>("use_motion_model", true);
  this->get_parameter("use_motion_model", using_motion_model_);

  this->declare_parameter<std::string>("inference_strategy","FixedLagSmoothing");
  this->get_parameter("inference_strategy", inference_strategy_);

  this->declare_parameter<bool>("init_from_ground_truth", true);
  this->get_parameter("init_from_ground_truth", init_from_ground_truth_);
  name_space_ = this->get_namespace();
  InferenceStrategy inference_strategy;
  if(inference_strategy_ == "ISAM2"){
    inference_strategy = InferenceStrategy::ISAM2;
  }
  else if(inference_strategy_ == "FixedLagSmoothing"){
    inference_strategy = InferenceStrategy::FixedLagSmoothing;
  }
  else if (inference_strategy_ == "EKF") {
    inference_strategy = InferenceStrategy::EKF;
  }
  else if (inference_strategy_ == "FullSmoothing") {
    inference_strategy = InferenceStrategy::FullSmoothing;
  }
  else {
    throw std::invalid_argument("Invalid inference strategy, choose between ISAM2, FixedLagSmoothing, EKF or FullSmoothing");
  }
  // // Subscriptions for sensors
  stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::STIM_IMU_TOPIC, 10,
      std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));

  sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::SBG_IMU_TOPIC, 10,
      std::bind(&StateEstimator::sbg_callback, this, std::placeholders::_1));

  dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
      sam_msgs::msg::Topics::DVL_TOPIC, 10,
      std::bind(&StateEstimator::dvl_callback, this, std::placeholders::_1));

  barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      sam_msgs::msg::Topics::PRESS_DEPTH20_TOPIC, 10,
      std::bind(&StateEstimator::barometer_callback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      smarc_msgs::msg::Topics::GPS_TOPIC, 10,
      std::bind(&StateEstimator::gps_callback, this, std::placeholders::_1));

  //Subscribe to gt odometry if init_from_ground_truth_ is true
  if(init_from_ground_truth_)
  {
    gt_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mocap/sam_mocap/odom", 10,
      std::bind(&StateEstimator::gt_odom_callback, this, std::placeholders::_1));
  }
  

  if(using_motion_model_)
  {
    // Subscription for contorl input.
  // thruster_vector_sub_ = this-> create_subscription<sam_msgs::msg::ThrusterAngles>(
  //     sam_msgs::msg::Topics::THRUST_VECTOR_CMD_TOPIC, 10,
  //      std::bind(&StateEstimator::ThrusterVectorCallback, this, std::placeholders::_1));

  // // Subscriptions for control inputs with message filters
  // thruster1_sub_.subscribe(this, sam_msgs::msg::Topics::THRUSTER1_CMD_TOPIC);
  // thruster2_sub_.subscribe(this, sam_msgs::msg::Topics::THRUSTER2_CMD_TOPIC);
  // lcg_sub_.subscribe(this, sam_msgs::msg::Topics::LCG_FB_TOPIC);
  // vbs_sub_.subscribe(this, sam_msgs::msg::Topics::VBS_FB_TOPIC);

    thruster_vector_sub_ = this->create_subscription<sam_msgs::msg::ThrusterAngles>(
      "/piml/thrust_vector_cmd", 10,
      std::bind(&StateEstimator::ThrusterVectorCallback, this, std::placeholders::_1));
    // thruster_sub_ = this->create_subscription<sam_msgs::msg::ThrusterRPMs>(
    //   "/piml/thrust_vector_cmd", 10,
    //   std::bind(&StateEstimator::thruster_callback, this, std::placeholders::_1));

    thruster1_sub_.subscribe(this, "/piml/thruster1_cmd");
    thruster2_sub_.subscribe(this, "/piml/thruster2_cmd");

    thruster_sync_ = std::make_shared<ThrusterSync>(
      ThrusterSyncPolicy(10),
      thruster1_sub_,
      thruster2_sub_
    );
    thruster_sync_->registerCallback(
      std::bind(&StateEstimator::thruster_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    lcg_sub_.subscribe(this, "/piml/lcg_fb");
    vbs_sub_.subscribe(this, "/piml/vbs_fb");

    lcg_vbs_sync_ = std::make_shared<LcgVbsSync>(
      LcgVbsSyncPolicy(10),
      lcg_sub_,
      vbs_sub_
    );
    lcg_vbs_sync_->registerCallback(
      std::bind(&StateEstimator::lcg_vbs_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );
  }

  // Use simulation time.
  this->set_parameter(rclcpp::Parameter("use_sim_time", true));
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);  

  // Publishers
  motion_model_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "motion_model_odom", 10);
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      dead_reckoning_msgs::msg::Topics::DR_ODOM_TOPIC, 10);
  // Timer for keyframe updates. This should be changed to a seperate thread
  KeyframeTimer = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&StateEstimator::KeyframeTimerCallback, this));

  // Initialize the GtsamGraph with the chosen inference strategy
  gtsam_graph_ = std::make_unique<GtsamGraph>(inference_strategy);
  pmm = std::make_unique<PreintegratedMotionModel>(dt_);
  //get the start time

}

void StateEstimator::gt_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (map_initialized_) return;
  
  geometry_msgs::msg::TransformStamped ned_to_enu;
  ned_to_enu.header.stamp    = this->get_clock()->now();
  ned_to_enu.header.frame_id = "mocap";          // NED 
  ned_to_enu.child_frame_id  = "map";            // ENU 

  ned_to_enu.transform.rotation.x = 0.70710678;
  ned_to_enu.transform.rotation.y = 0.70710678;
  ned_to_enu.transform.rotation.z = 0.0;
  ned_to_enu.transform.rotation.w = 0.0;

  tf_static_broadcaster_->sendTransform(ned_to_enu);
  geometry_msgs::msg::TransformStamped map_to_blgt;
  try {
      map_to_blgt = tf_buffer_.lookupTransform(
          "map",                     
          "sam/base_link",
          tf2::TimePointZero,     
          tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed: %s", ex.what());
      return;                                 // try again next packet
  }
  map_to_blgt.header.frame_id = "map";
  map_to_blgt.child_frame_id  = "odom";
  // geometry_msgs::msg::Vector3 t_enu;
  // t_enu.x =  msg->pose.pose.position.y;   
  // t_enu.y =  msg->pose.pose.position.x;   
  // t_enu.z = -msg->pose.pose.position.z;   

  // tf2::Quaternion q_ned;
  // tf2::fromMsg(msg->pose.pose.orientation, q_ned);

  // static const tf2::Quaternion q_ned2enu(0.70710678, 0.70710678, 0, 0);
  // tf2::Quaternion q_enu = q_ned2enu * q_ned;
  // q_enu.normalize();
  // geometry_msgs::msg::TransformStamped map_to_odom;
  // map_to_odom.header.stamp    = msg->header.stamp;
  // map_to_odom.header.frame_id = "map";
  // map_to_odom.child_frame_id  = "odom";
  // map_to_odom.transform.translation = t_enu;
  // map_to_odom.transform.rotation    = tf2::toMsg(q_enu);
  tf2::Quaternion q = tf2::Quaternion(map_to_blgt.transform.rotation.x,
                                     map_to_blgt.transform.rotation.y,
                                     map_to_blgt.transform.rotation.z,
                                     map_to_blgt.transform.rotation.w);
  // tf2::Quaternion q_ned_to_enu; 
  tf2::Quaternion q_ned_to_enu;
  q_ned_to_enu.setRPY(M_PI, 0.0, 0.0);     
  tf2::Quaternion q_enu =  q * q_ned_to_enu ;
  q_enu.normalize();
  map_to_blgt.transform.rotation.x = q_enu.x();
  map_to_blgt.transform.rotation.y = q_enu.y();
  map_to_blgt.transform.rotation.z = q_enu.z();
  map_to_blgt.transform.rotation.w = q_enu.w();


  gt_init_quat_ = gtsam::Quaternion(q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
  tf_static_broadcaster_->sendTransform(map_to_blgt);
  // RCLCPP_INFO(this->get_logger(),"quat %f %f %f %f", q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
  map_initialized_ = true;
  gt_pose_sub_.reset();                      
  
}


void StateEstimator::ThrusterVectorCallback(const sam_msgs::msg::ThrusterAngles::SharedPtr msg)
{
  if(is_graph_initialized_)
  {
  Eigen::VectorXd u(2);
  u << msg->thruster_vertical_radians,
       msg->thruster_horizontal_radians;
      
  double timestamp = rclcpp::Time(msg->header.stamp).seconds();
  pmm -> controlToList(u,timestamp,true);
  RCLCPP_INFO(this->get_logger(), "Thruster vector callback: %f %f", msg->thruster_vertical_radians, msg->thruster_horizontal_radians);
  }
}

// void StateEstimator::control_input_callback(const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr thruster1,
//                                             const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr thruster2,
//                                             const smarc_msgs::msg::PercentStamped::ConstSharedPtr lcg,
//                                             const smarc_msgs::msg::PercentStamped::ConstSharedPtr vbs) {

//   if(is_graph_initialized_){
//     RCLCPP_INFO(this->get_logger(), "Control input received");
//     Eigen::VectorXd u_test(4);
//     u_test <<lcg->value, vbs->value, thruster1->rpm, thruster2->rpm;
//     rclcpp::Time time(lcg->header.stamp);
//     double timestamp = time.seconds();
//     pmm -> controlToList(u_test,timestamp,false);
//   }


// }

// thrusters-only
void StateEstimator::thruster_callback(const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t1,
                                        const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t2)
{
  if(is_graph_initialized_)
  {
  last_thr1_rpm_ = t1->rpm;
  last_thr2_rpm_ = t2->rpm;

  Eigen::Vector4d u_fb;
  u_fb << last_lcg_,      // from previous lcg/vbs callback
          last_vbs_,
          last_thr1_rpm_,
          last_thr2_rpm_;
  RCLCPP_INFO(this->get_logger(), "Thruster callback: %f %f %f %f", last_lcg_, last_vbs_, last_thr1_rpm_, last_thr2_rpm_);
   double timestamp = rclcpp::Time(t1->header.stamp).seconds();
   pmm->controlToList(u_fb, timestamp, false);
  }
}

// LCG/VBS-only
void StateEstimator::lcg_vbs_callback(
  const smarc_msgs::msg::PercentStamped::ConstSharedPtr lcg,
  const smarc_msgs::msg::PercentStamped::ConstSharedPtr vbs)
{
  if(is_graph_initialized_)
  {
  last_lcg_ = lcg->value;
  last_vbs_ = vbs->value;

  Eigen::Vector4d u_fb;
  u_fb << last_lcg_,      
          last_vbs_,
          last_thr1_rpm_,// from previous thruster callback
          last_thr2_rpm_;
  RCLCPP_INFO(this->get_logger(), "LCG/VBS callback: %f %f %f %f", last_lcg_, last_vbs_, last_thr1_rpm_, last_thr2_rpm_);
   double timestamp = rclcpp::Time(lcg->header.stamp).seconds();
   pmm->controlToList(u_fb, timestamp, false);
  }
}



void StateEstimator::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z()); // Adjusted gyro measurements to right-hand rule.

  double delta_t = 1.0 / 125.0;
  gtsam_graph_->integrateImuMeasurement(acc, gyro, delta_t);
 }




void StateEstimator::sbg_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);

  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);
  Vector3 sbg_gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z());
  if(number_of_imu_measurements< 6)
  {
    Rot3 current_rotation = Rot3::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    estimated_rotations_.push_back(current_rotation);
    number_of_imu_measurements++;
  }
  double delta_t = 1.0 / 50.0; // or use: current_time - last_time_;
  gtsam_graph_->integrateSbgMeasurement(acc, sbg_gyro, delta_t);
}


void StateEstimator::dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg)
{
    Vector3 vel_dvl(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    latest_dvl_measurement_ = vel_dvl;
    dvl_gyro = gyro;
    new_dvl_measurement_ = true;
}



void StateEstimator::barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  double measured_pressure = msg->fluid_pressure;
  double depth = -(measured_pressure - atmospheric_pressure_) / 9806.65; //Down negative
  if(map_initialized_ && is_graph_initialized_){
    if (!baro_calibrated) {
    gtsam::Vector3 base_to_pressure_offset(-0.503, 0.025, 0.057);

    // geometry_msgs::msg::TransformStamped tf_blenu;
    // try {
    //     tf_blenu = tf_buffer_.lookupTransform("sam_auv_v1/odom_gt","sam_auv_v1/base_link_gt",msg->header.stamp,std::chrono::seconds(1));
    //   } catch (tf2::TransformException &ex) {
    //     RCLCPP_WARN(this->get_logger(),
    //                 "Could not get base_link→odom: %s", ex.what());
    //     return;
    //   }
    //   tf2::Quaternion q_blenu;
    //   tf2::fromMsg(tf_blenu.transform.rotation, q_blenu);
    //   tf2::Vector3 sensor_offset = tf2::quatRotate(q_blenu, base_to_pressure_offset);
    gtsam::Vector3 sensor_offset = previous_state_.rotation().matrix().transpose()*base_to_pressure_offset;
      double z_sensor_enu = sensor_offset.z(); // this is the offset from the base_link to the pressure sensor in ENU frame
      static_offset_ = z_sensor_enu - depth; // this is the offset to the static frame
      baro_calibrated = true;
    }
  latest_depth_measurement_ =  depth + static_offset_; // depth in the odom frame
  RCLCPP_INFO(this->get_logger(), "Depth: %f", latest_depth_measurement_);
  new_barometer_measurement_received_ = true;
  }
}



void StateEstimator::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if(init_from_ground_truth_) return;
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
  RCLCPP_WARN(this->get_logger(), "Received GPS message without valid fix (status: %d)", msg->status.status);
  return;
}

  double utm_x, utm_y, utm_z;
  if(!map_initialized_ ){
  // if sim time is used, take the ground truth as gps reading
  if(this->get_parameter("use_sim_time").as_bool()){
    try{
      transformStamped = tf_buffer_.lookupTransform("utm_34_V", "sam_auv_v1/gps_link_gt",
                                                      tf2::TimePointZero, std::chrono::seconds(1));
      utm_x = transformStamped.transform.translation.x;
      utm_y = transformStamped.transform.translation.y;
      utm_z = transformStamped.transform.translation.z;
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
    }
    
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }
  }
  // if not using the sim, take the real gps reading
  else {
      double var = msg->position_covariance[0];
  if (var > cov_threshold_*cov_threshold_) {
    RCLCPP_WARN(get_logger(),
    "GPS covariance too high (sigma=%.1f m), dropping fix", std::sqrt(var));
    return;
  }
    sum_lat_ += msg->latitude;
    sum_lon_ += msg->longitude;
    sum_alt_ += msg->altitude;
    number_of_gps_measurements_++;
   
   if (number_of_gps_measurements_ >= number_of_gps_measurements_for_map_init_) {
    double avg_lat = sum_lat_  / number_of_gps_measurements_;
    double avg_lon = sum_lon_  / number_of_gps_measurements_;
    double avg_alt = sum_alt_  / number_of_gps_measurements_;
    int utm_zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(avg_lat, avg_lon, utm_zone, northp, utm_x, utm_y);
    utm_z = avg_alt;





    // Create a static transform from "utm" to "map" using the UTM coordinates.
    geometry_msgs::msg::TransformStamped map_transform;
    map_transform.header.stamp = this->get_clock()->now();
    map_transform.header.frame_id = "utm_34_V";     // Frmae name from sim
    map_transform.child_frame_id = "map";        
    map_transform.transform.translation.x = utm_x;
    map_transform.transform.translation.y = utm_y;
    map_transform.transform.translation.z = 0;
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
 return;
}
return;
  }
  // Convert the GPS coordinates to UTM coordinates
  //   int utm_zone;
  //   bool northp;
  //   GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone, northp, utm_x, utm_y);
  //   utm_z = msg->altitude;
  // // Compare the new gps message with the first one to get the offset, but we need it in the odom frame
  // if(is_graph_initialized_){
  //   Point3 map_to_odom_offset;
  //   Rot3 map_to_odom_rotation;
  //   try{
  //     transformStamped = tf_buffer_.lookupTransform("map", "odom",
  //                                                     tf2::TimePointZero, std::chrono::seconds(1));
  //     map_to_odom_offset = Point3(transformStamped.transform.translation.x,
  //                                 transformStamped.transform.translation.y,
  //                                 transformStamped.transform.translation.z);
  //     map_to_odom_rotation = Rot3(transformStamped.transform.rotation.w,
  //                                 transformStamped.transform.rotation.x,
  //                                 transformStamped.transform.rotation.y,
  //                                 transformStamped.transform.rotation.z);

  //   }
  //   catch (tf2::TransformException &ex) {
  //     RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  //     return;
  //   }
  //   Point3 gps_in_map(utm_x - first_utm_x, utm_y - first_utm_y, utm_z - first_utm_z);
  //   // Apply rotation from map to odom
  //   Point3 gps_in_odom = map_to_odom_rotation.inverse().rotate(gps_in_map - map_to_odom_offset);
  //   latest_gps_point_ = gps_in_odom;
  //   new_gps_measurement_ = true;
  //   // Logg off the gps point of the gps in the odom frame
  //   RCLCPP_DEBUG(this->get_logger(), "GPS Point: [%f, %f, %f]", latest_gps_point_.x(), latest_gps_point_.y(), latest_gps_point_.z());
  // }
 
}  



Rot3 StateEstimator::averageRotations(const std::vector<Rot3>& rotations) {
  Vector3 sumLog = Vector3::Zero();
  for (const auto& rot : rotations) {
    sumLog += Rot3::Logmap(rot);
  }
  Vector3 avgLog = sumLog / static_cast<double>(rotations.size());
  return Rot3::Expmap(avgLog);
}





void StateEstimator::KeyframeTimerCallback(){
  // need to have at least 6 imu measurements to initialize the graph with the current orientation
  double current_time = this->get_clock()->now().seconds();
  gtsam_graph_-> setTimeStamp(current_time);
  auto t1 = std::chrono::high_resolution_clock::now();
    if(number_of_imu_measurements < 6){
          RCLCPP_INFO(get_logger(),
      "  skipping: only %d IMUs (need ≥6)", number_of_imu_measurements);
      return;
      }
    if(!map_initialized_){
       RCLCPP_INFO(get_logger(), "  skipping: map_initialized_ == false");
      return;
    }
    if (!is_graph_initialized_) {
        Quaternion initial_quat;
        if(init_from_ground_truth_)
        {
          //no rotation
          initial_quat = gtsam::Quaternion(1.0, 0.0, 0.0, 0.0); 
        }
        else{
        //  if(this->get_parameter("use_sim_time").as_bool()){
        //   try {
        //   // 1) get the base_link in UTM coords
        //   auto tf_base = tf_buffer_.lookupTransform(
        //       "sam_auv_v1/gps_link_gt",               // source frame
        //       "sam_auv_v1/base_link_gt",          // target frame
        //       tf2::TimePointZero,
        //       std::chrono::seconds(1));
        //   tf2::Quaternion q_utm;
        //   tf2::fromMsg(tf_base.transform.rotation, q_utm);
        //   double utm_bx = tf_base.transform.translation.x;
        //   double utm_by = tf_base.transform.translation.y;
        //   double utm_bz = tf_base.transform.translation.z;
        //   tf2::Quaternion q_ned_to_enu;
        //   q_ned_to_enu.setRPY(/*roll*/M_PI, /*pitch=*/0.0, /*yaw=*/M_PI/2.0);

        //   tf2::Quaternion q_enu = q_ned_to_enu * q_utm;
        //   q_enu.normalize();
        //   // 2) convert into your 'map' frame by subtracting the very first GPS fix
        //   double mx = utm_bx - first_utm_x;
        //   double my = utm_by - first_utm_y;
        //   double mz = utm_bz - first_utm_z;

        //   // 3) build the map→odom transform
        //   geometry_msgs::msg::TransformStamped map_to_odom;
        //   map_to_odom.header.stamp    = this->get_clock()->now();
        //   map_to_odom.header.frame_id = "map";
        //   map_to_odom.child_frame_id  = "odom";

        //   map_to_odom.transform.translation.x = mx;
        //   map_to_odom.transform.translation.y = my;
        //   map_to_odom.transform.translation.z = mz;

        //   // use the exact base_link orientation in UTM as your starting rotation
        //   map_to_odom.transform.rotation = tf2::toMsg(q_enu);

        //   tf_static_broadcaster_->sendTransform(map_to_odom);
        //   RCLCPP_INFO(this->get_logger(),
        //               "Initialized odom at map (%.3f, %.3f, %.3f)",
        //               mx, my, mz);

        //   // svae initail orientaiton as gtsam quat
        //   initial_quat = Quaternion(q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());

        // }
        // catch (tf2::TransformException &ex) {
        //   RCLCPP_WARN(this->get_logger(),
        //               "Could not lookup base_link in UTM: %s",
        //               ex.what());
        // }
        //  }
        // else{
        Rot3 average_rotation = averageRotations(estimated_rotations_);
        Quaternion quat = average_rotation.toQuaternion();
        // Initialize the odom frame from map
        Point3 base_to_gps_offset(0.528 ,0.0, 0.071);
        tf2::Quaternion q_tf2( quat.x(), quat.y(), quat.z(), quat.w());

        tf2::Vector3 off_base(base_to_gps_offset.x(),base_to_gps_offset.y(),base_to_gps_offset.z()  );
        tf2::Vector3 off_map = tf2::quatRotate(q_tf2, off_base);
        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.header.stamp = this->get_clock()->now();
        odom_transform.header.frame_id = "map";
        odom_transform.child_frame_id = "odom";
        // translate the map -> odom with -base_to_gps_offset in x and y
        odom_transform.transform.translation.x = -off_map.x(); // x and y were swapped from the sim
        odom_transform.transform.translation.y = -off_map.y();
        odom_transform.transform.translation.z = -off_map.z();
        Rot3 map_to_odom_rot = average_rotation;
        Quaternion map_to_odom_quat = map_to_odom_rot.toQuaternion();
        odom_transform.transform.rotation.x = 0;
        odom_transform.transform.rotation.y = 0;
        odom_transform.transform.rotation.z = 0;
        odom_transform.transform.rotation.w = 1;
        //no roation for initial quat
        initial_quat = gtsam::Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
        tf_static_broadcaster_->sendTransform(odom_transform);
        RCLCPP_INFO(this->get_logger(),
                    "Initialized odom at map (%.3f, %.3f, %.3f)",
                    odom_transform.transform.translation.x,
                    odom_transform.transform.translation.y,
                    odom_transform.transform.translation.z);
        // }
        }
        // TODO take the orientation of the compass as the initial orientation
        // start the graph with average orientation from the imu,
       
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
        
        init_transform.transform.rotation.x = initial_quat.x();
        init_transform.transform.rotation.y = initial_quat.y();
        init_transform.transform.rotation.z = initial_quat.z();
        init_transform.transform.rotation.w = initial_quat.w();
        tf_broadcast_.sendTransform(init_transform);

        // Initialize the GTSAM graph and state.
        gtsam_graph_->initGraphAndState(initial_quat, initial_position);
        RCLCPP_INFO(this->get_logger(),"Graph initialized !!!");
        previous_state_ = gtsam_graph_->getCurrentState();
        is_graph_initialized_ = true;
        current_time = this->get_clock()->now().seconds();
        last_time_ = current_time;
        // new_current_integration_state_ = true;
        return;
      
    }
    double dt = gtsam_graph_->getTij(); 
    if (dt <= 0) {
      RCLCPP_DEBUG(get_logger(), "No new IMU data this cycle, skipping IMU factor + optimize");
      return;
    }
    // gtsam_graph_->incrementIndex();
    if(using_motion_model_){
      NavState state = NavState(previous_state_.pose(), previous_state_.velocity());

      NavState new_state = pmm->predict(state, gyro, last_time_, current_time);
      gtsam_graph_->addMotionModelFactor(last_time_,current_time,pmm,gyro,new_state);
      last_time_ = current_time;
    nav_msgs::msg::Odometry motion_model_odom;
    motion_model_odom.header.stamp = this->get_clock()->now();
    motion_model_odom.header.frame_id = "odom";
    motion_model_odom.child_frame_id = "base_link";

    motion_model_odom.pose.pose.position.x = new_state.pose().translation().x();
    motion_model_odom.pose.pose.position.y = new_state.pose().translation().y();
    motion_model_odom.pose.pose.position.z = new_state.pose().translation().z();
    Quaternion quat = new_state.pose().rotation().toQuaternion();
    motion_model_odom.pose.pose.orientation.x = quat.x();
    motion_model_odom.pose.pose.orientation.y = quat.y();
    motion_model_odom.pose.pose.orientation.z = quat.z();
    motion_model_odom.pose.pose.orientation.w = quat.w();
    motion_model_odom.twist.twist.linear.x = new_state.velocity().x();
    motion_model_odom.twist.twist.linear.y = new_state.velocity().y();
    motion_model_odom.twist.twist.linear.z = new_state.velocity().z();
    motion_model_odom_->publish(motion_model_odom);
    }
    
    // Predict the next state using the preintegrated measurements AND add the imu factor to the graph.
    // NavState predictes_imu_state = gtsam_graph_->addImuFactor();
    // RCLCPP_INFO(this->get_logger(), "IMU prediction state: [%f, %f, %f]",
    //             predictes_imu_state.pose().translation().x(),
    //             predictes_imu_state.pose().translation().y(),
    //             predictes_imu_state.pose().translation().z());
    // NavState predicted_sbg_state = gtsam_graph_->addSbgFactor();
    // RCLCPP_INFO(this->get_logger(), "SBG prediction state: [%f, %f, %f]",
    //             predicted_sbg_state.pose().translation().x(),
    //             predicted_sbg_state.pose().translation().y(),
    //             predicted_sbg_state.pose().translation().z());

   
    // // Add the DVL, GPS and Barometer factors to the graph.
    // if (new_dvl_measurement_) {  
    //   gtsam_graph_->addDvlFactor(latest_dvl_measurement_, dvl_gyro );
    //   new_dvl_measurement_ = false;
    // }

    // if (new_gps_measurement_) {
    //   gtsam_graph_->addGpsFactor(latest_gps_point_);
    //   new_gps_measurement_ = false;
    // }

    // if (new_barometer_measurement_received_) {
    //   gtsam_graph_->addBarometerFactor(latest_depth_measurement_);
    //   // RCLCPP_INFO(this->get_logger(), "Barometer Factor Added for Depth: %f", latest_depth_measurement_);
    //   new_barometer_measurement_received_ = false;
    // }

    // Optimize the factor graph.
    gtsam_graph_->optimize();
    // pmm->resetIntegration();
    // new_current_integration_state_ = true;
    // Update the current state and bias.
    if(using_motion_model_){
      pmm->resetIntegration();
    }
    // RCLCPP_INFO(this->get_logger(), "Resetting the integration");
    current_imu_bias_ = gtsam_graph_->getCurrentImuBias();
    // current_sbg_bias_ = gtsam_graph_->getCurrentSbgBias();
    previous_state_ = gtsam_graph_->getCurrentState();
    //log the current state
    RCLCPP_INFO(this->get_logger(), "Current State: [%f, %f, %f]",
                previous_state_.pose().translation().x(),
                previous_state_.pose().translation().y(),
                previous_state_.pose().translation().z());
   
    // Publish the estimated pose.
    nav_msgs::msg::Odometry estimated_pose;
    estimated_pose.header.stamp = this->get_clock()->now();
    estimated_pose.header.frame_id = "odom";
    estimated_pose.child_frame_id = "base_link";
    estimated_pose.pose.pose.position.x = previous_state_.pose().translation().x();
    estimated_pose.pose.pose.position.y = previous_state_.pose().translation().y();
    estimated_pose.pose.pose.position.z = previous_state_.pose().translation().z();
    Quaternion quat = previous_state_.pose().rotation().toQuaternion();
    estimated_pose.pose.pose.orientation.x = quat.x();
    estimated_pose.pose.pose.orientation.y = quat.y();
    estimated_pose.pose.pose.orientation.z = quat.z();
    estimated_pose.pose.pose.orientation.w = quat.w();
    estimated_pose.twist.twist.linear.x = previous_state_.velocity().x();
    estimated_pose.twist.twist.linear.y = previous_state_.velocity().y();
    estimated_pose.twist.twist.linear.z = previous_state_.velocity().z();
    pose_pub_->publish(estimated_pose);
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

    auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_time = t2 - t1;
  std::cout << "Optimization took " << elapsed_time.count() << " seconds." << std::endl;

}



int main(int argc, char **argv) {
  py::scoped_interpreter guard{};
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
