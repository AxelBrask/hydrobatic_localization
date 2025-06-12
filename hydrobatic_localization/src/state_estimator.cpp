#include "hydrobatic_localization/state_estimator.h"


StateEstimator::StateEstimator()
  : Node("state_estimator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    tf_broadcast_(this), number_of_imu_measurements(0), is_graph_initialized_(false),
    new_dvl_measurement_(false), new_gps_measurement_(false), map_initialized_(false),
    first_barometer_measurement_(0.0), new_barometer_measurement_received_(false), atmospheric_pressure_(100800.0),
    dt_(0.01)
{

  // Declare parameters
  this->declare_parameter<bool>("use_motion_model", true);
  this->get_parameter("use_motion_model", using_motion_model_);

  this->declare_parameter<std::string>("inference_strategy","FixedLagSmoothing");
  this->get_parameter("inference_strategy", inference_strategy_);

  this->declare_parameter<bool>("init_from_ground_truth", true);
  this->get_parameter("init_from_ground_truth", init_from_ground_truth_);

  this->declare_parameter<std::string>("config_file", "sam.yaml");
  this->get_parameter("config_file", config_file_);

  this->declare_parameter<int>("kf_interval_hz", 10);
  this->get_parameter("kf_interval_hz", kf_interval_hz_);
  bool use_sim_time_;
  // this->declare_parameter<bool>("use_sim_time", false);
  this->get_parameter("use_sim_time", use_sim_time_);
  this->declare_parameter<bool>("use_sensor_covariance", false);
  this->get_parameter("use_sensor_covariance", use_sensor_covariance_);

  std::string config_file;
  if (std::filesystem::path(config_file_).is_absolute()) {
    config_file = config_file_;
  } else {
    auto pkg_share = ament_index_cpp::get_package_share_directory("hydrobatic_localization");
    config_file = pkg_share + "/config/" + config_file_;
  }

  RCLCPP_INFO(this->get_logger(), "Loading config from %s", config_file.c_str());
  //logg the ros parameters
  RCLCPP_INFO(this->get_logger(), "Using motion model: %s", using_motion_model_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Init from ground truth: %s", init_from_ground_truth_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Use sensor covariance: %s", use_sensor_covariance_ ? "true" : "false");
  name_space_ = this->get_namespace();
  //remove leading slashes from namespace
  if (name_space_.front() == '/') {
    name_space_.erase(0, 1);
  }
  std::cout << "Namespace: " << name_space_ << std::endl;
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
  // Subscriptions for sensors
  stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::STIM_IMU_TOPIC, 100,
      std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));

  sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      sam_msgs::msg::Topics::SBG_IMU_TOPIC, 100,
      std::bind(&StateEstimator::sbg_callback, this, std::placeholders::_1));

   dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
        sam_msgs::msg::Topics::DVL_TOPIC, 10, /*use "/sam/core/dvl_3beams" for real sam otherwise use */
      std::bind(&StateEstimator::dvl_callback, this, std::placeholders::_1));

  barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      sam_msgs::msg::Topics::PRESS_DEPTH300_TOPIC, 10,     /*If sim: use depth20 on real sam use depth300*/
      std::bind(&StateEstimator::barometer_callback, this, std::placeholders::_1));


  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
  smarc_msgs::msg::Topics::GPS_TOPIC, 10,
  std::bind(&StateEstimator::gps_callback, this, std::placeholders::_1));

  // depth_pub_ = this ->create_publisher<geometry_msgs::msg::PoseStamped>(
  //     "depth", 10);

  // gt_pressure_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  //     "gt_pressure_depth", 10);
  //Subscribe to gt odometry if init_from_ground_truth_ is true
  if(init_from_ground_truth_)
  {
    gt_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mocap/sam_mocap2/odom", 10,
      std::bind(&StateEstimator::gt_odom_callback, this, std::placeholders::_1));
  }
  
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "gt_twist", 10);

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mocap/sam_mocap2/velocity", 10,
      std::bind(&StateEstimator::gt_velocity_callback, this, std::placeholders::_1)); 

  if(using_motion_model_)
  {
    thruster_vector_sub_ = this->create_subscription<sam_msgs::msg::ThrusterAngles>(
      "piml/thrust_vector_cmd", 10,
      std::bind(&StateEstimator::ThrusterVectorCallback, this, std::placeholders::_1));

    thruster1_sub_.subscribe(this, "piml/thruster1_cmd");
    thruster2_sub_.subscribe(this, "piml/thruster2_cmd");

    thruster_sync_ = std::make_shared<ThrusterSync>(
      ThrusterSyncPolicy(10), thruster1_sub_, thruster2_sub_);

    thruster_sync_->registerCallback(
      std::bind(&StateEstimator::thruster_callback, this, std::placeholders::_1, std::placeholders::_2));

    lcg_sub_.subscribe(this, "piml/lcg_fb");
    vbs_sub_.subscribe(this, "piml/vbs_fb");

    lcg_vbs_sync_ = std::make_shared<LcgVbsSync>(
      LcgVbsSyncPolicy(10), lcg_sub_, vbs_sub_);

    lcg_vbs_sync_->registerCallback(
      std::bind(&StateEstimator::lcg_vbs_callback, this, std::placeholders::_1, std::placeholders::_2) );
  }

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);  

  // Publishers
  motion_model_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "motion_model_odom", 10);
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      dead_reckoning_msgs::msg::Topics::DR_ODOM_TOPIC, 10);

  KeyframeTimer = this->create_wall_timer(
      std::chrono::milliseconds(1000/kf_interval_hz_), std::bind(&StateEstimator::KeyframeTimerCallback, this));
  RCLCPP_INFO(this->get_logger(), "Keyframe timer set to %d Hz", kf_interval_hz_);

  // Initialize the GtsamGraph with the chosen inference strategy
  gtsam_graph_ = std::make_unique<GtsamGraph>(inference_strategy, config_file);
  pmm = std::make_unique<PreintegratedMotionModel>(dt_);

  std::random_device rd;
  noise_generator_ = std::default_random_engine(rd());
  double sigma_lin = 0.05;   // set the noise of the gt vels to whaterver you want
  noise_lin_x_ = std::normal_distribution<double>(0.0, sigma_lin);
  noise_lin_y_ = std::normal_distribution<double>(0.0, sigma_lin);
  noise_lin_z_ = std::normal_distribution<double>(0.0, sigma_lin);

}

void StateEstimator::gt_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  geometry_msgs::msg::VelocityStamped vel_mocap;
  vel_mocap.header = msg->header;
  vel_mocap.velocity = msg->twist;    

  geometry_msgs::msg::TransformStamped T_odom_from_base;
  try {
    T_odom_from_base = tf_buffer_.lookupTransform(
      "sam_mocap2/base_link",                     
      vel_mocap.header.frame_id, 
      rclcpp::Time(0),           
      tf2::durationFromSec(0.1)  
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Quaternion q_odom_from_base;
  tf2::fromMsg(T_odom_from_base.transform.rotation, q_odom_from_base);
  tf2::Matrix3x3 R_odom_from_mocap(q_odom_from_base);

  tf2::Vector3 v_base(
    vel_mocap.velocity.linear.x,
    vel_mocap.velocity.linear.y,
    vel_mocap.velocity.linear.z
  );
  tf2::Vector3 v_odom = R_odom_from_mocap * v_base;

  tf2::Vector3 w_base(
    vel_mocap.velocity.angular.x,
    vel_mocap.velocity.angular.y,
    vel_mocap.velocity.angular.z
  );
  tf2::Vector3 w_odom = R_odom_from_mocap * w_base;



  geometry_msgs::msg::TwistStamped vel_odom;
  vel_odom.header.stamp = vel_mocap.header.stamp;
  vel_odom.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK; // Odom frame in ENU
  vel_odom.twist.linear.x  = v_odom.x();
  vel_odom.twist.linear.y  = v_odom.y();
  vel_odom.twist.linear.z  = v_odom.z();
  vel_odom.twist.angular.x = w_odom.x();
  vel_odom.twist.angular.y = w_odom.y();
  vel_odom.twist.angular.z = w_odom.z();
  double noisy_lin_x = v_odom.x() + noise_lin_x_(noise_generator_);
  double noisy_lin_y = -v_odom.y()+ noise_lin_y_(noise_generator_);
  double noisy_lin_z = -v_odom.z()+ noise_lin_z_(noise_generator_);
  gt_velocity_ = gtsam::Vector3(noisy_lin_x, noisy_lin_y, noisy_lin_z);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK;
  odom_msg.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK;

  odom_msg.twist.twist.linear.x = v_odom.x();
  odom_msg.twist.twist.linear.y = v_odom.y();
  odom_msg.twist.twist.linear.z = v_odom.z();
  odom_msg.twist.twist.angular.x = w_odom.x();
  odom_msg.twist.twist.angular.y = w_odom.y();
  odom_msg.twist.twist.angular.z = w_odom.z();
  motion_model_odom_->publish(odom_msg);
}


// Callback for the ground truth odometry in order to align the initial odom frame with gt
void StateEstimator::gt_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::VelocityStamped vel_in;
  vel_in.header  = msg->header;             
  vel_in.velocity = msg->twist.twist;       
  if (!map_initialized_)
  {
  
      geometry_msgs::msg::TransformStamped ned_to_enu;
      ned_to_enu.header.stamp    = this->get_clock()->now();
      ned_to_enu.header.frame_id = "mocap";           
      ned_to_enu.child_frame_id  = "map";             

      // 180° rotation about X to go from NED to ENU
      ned_to_enu.transform.rotation.x = 0.70710678;
      ned_to_enu.transform.rotation.y = 0.70710678;
      ned_to_enu.transform.rotation.z = 0.0;
      ned_to_enu.transform.rotation.w = 0.0;

      tf_static_broadcaster_->sendTransform(ned_to_enu);
      RCLCPP_INFO(this->get_logger(), "NED→ENU static transform published");

      geometry_msgs::msg::TransformStamped map_to_blgt;
      try {
        map_to_blgt = tf_buffer_.lookupTransform(
          "map",                     
          "sam_mocap2/base_link",
          tf2::TimePointZero,     
          tf2::durationFromSec(0.5));
      } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;                                 
      }
      map_to_blgt.header.frame_id = "map";
      map_to_blgt.child_frame_id  = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK;
  tf2::Quaternion q = tf2::Quaternion(map_to_blgt.transform.rotation.x,
                                     map_to_blgt.transform.rotation.y,
                                     map_to_blgt.transform.rotation.z,
                                     map_to_blgt.transform.rotation.w);
  

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

  geometry_msgs::msg::TransformStamped body_to_odom_init;
  try {
    body_to_odom_init = tf_buffer_.lookupTransform(
      name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK,                     
      vel_in.header.frame_id, 
      rclcpp::Time(0),            
      tf2::durationFromSec(0.1)   
    );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

      tf2::Quaternion q_body_to_odom_init;
      tf2::fromMsg(body_to_odom_init.transform.rotation, q_body_to_odom_init);
      tf2::Matrix3x3 R_body_to_odom_init(q_body_to_odom_init);

      // Rotate linear velocity:
      const auto & lin_in_init = vel_in.velocity.linear;
      tf2::Vector3 v_body_init(lin_in_init.x, lin_in_init.y, lin_in_init.z);
      tf2::Vector3 v_odom_init = R_body_to_odom_init * v_body_init;

      // Rotate angular velocity (if desired):
      const auto & ang_in_init = vel_in.velocity.angular;
      tf2::Vector3 w_body_init(ang_in_init.x, ang_in_init.y, ang_in_init.z);
      tf2::Vector3 w_odom_init = R_body_to_odom_init * w_body_init;

      init_vel_odom_.header.stamp    = vel_in.header.stamp;
      init_vel_odom_.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK;
      init_vel_odom_.velocity.linear.x  = v_odom_init.x();
      init_vel_odom_.velocity.linear.y  = v_odom_init.y();
      init_vel_odom_.velocity.linear.z  = v_odom_init.z();
      init_vel_odom_.velocity.angular.x = w_odom_init.x();
      init_vel_odom_.velocity.angular.y = w_odom_init.y();
      init_vel_odom_.velocity.angular.z = w_odom_init.z();



      map_initialized_ = true;
      return;
    }  

    //
    geometry_msgs::msg::TransformStamped body_to_odom;
    try {
      body_to_odom = tf_buffer_.lookupTransform(
        name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK,                   
        msg->child_frame_id,          
        tf2::TimePointZero,           
        tf2::durationFromSec(0.1)     
      );
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF lookup (odom←base_link) failed: %s", ex.what());
      return;
    }
    tf2::Quaternion q_body_to_odom;
    // Construct rotation matrix from quaternion
    tf2::fromMsg(body_to_odom.transform.rotation, q_body_to_odom);
    tf2::Matrix3x3 R_body_to_odom(q_body_to_odom);

    // Rotate linear velocity:
    const auto & lin = vel_in.velocity.linear;
    tf2::Vector3 v_body(lin.x, lin.y, lin.z);
    tf2::Vector3 v_odom = R_body_to_odom * v_body;

    // Rotate angular velocity if needed:
    const auto & ang = vel_in.velocity.angular;
    tf2::Vector3 w_body(ang.x, ang.y, ang.z);
    tf2::Vector3 w_odom = R_body_to_odom * w_body;

    const auto & T = body_to_odom.transform.translation;
    const auto & R_msg = body_to_odom.transform.rotation;
    gtsam::Pose3 pose_in_odom(
      gtsam::Rot3::Quaternion(R_msg.w, R_msg.x, R_msg.y, R_msg.z),
      gtsam::Point3(T.x, T.y, T.z)
    );
    gtsam::Vector3 vel_vec(v_odom.x(), v_odom.y(), v_odom.z());
    // gt_navstate_ = gtsam::NavState(pose_in_odom, vel_vec);
    //rotate with 180 roll
    gtsam::Rot3 R_enu_to_ned = gtsam::Rot3::RzRyRx(M_PI, 0.0, 0.0); // 180° roll to go from NED to ENU in body
    gtsam::Rot3 R_ned = pose_in_odom.rotation().compose(R_enu_to_ned);
    gtsam::Point3 T_ned(pose_in_odom.translation().x(),
                        pose_in_odom.translation().y(),
                        pose_in_odom.translation().z());
    gtsam::Pose3 enu_pose(R_ned, T_ned);
    gt_pose_ = enu_pose; // Store the pose in ENU  odom frame from mocap

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
  }
}
// thrusters-only
void StateEstimator::thruster_callback(const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t1,
                                        const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t2)
{
  if(is_graph_initialized_)
  {
    last_thr1_rpm_ = t1->rpm;
    last_thr2_rpm_ = t2->rpm;

    Eigen::Vector4d u_fb;
    u_fb << last_lcg_,      
            last_vbs_,
            last_thr1_rpm_,
            last_thr2_rpm_;
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
    u_fb << last_lcg_,last_vbs_, last_thr1_rpm_, last_thr2_rpm_;
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
  acc = Vector3(acc.x(), acc.y(), acc.z());
  gyro = Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z()); // Adjusted gyro measurements to right-hand rule.
  gtsam_graph_->integrateImuMeasurement(acc, gyro, gtsam_graph_->getImuRate());
 }




void StateEstimator::sbg_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  Vector3 acc(msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);

  Vector3 gyro_raw(msg->angular_velocity.x,
                   msg->angular_velocity.y,
                   msg->angular_velocity.z);  

  Vector3 sbg_gyro = Vector3(gyro_raw.x(), gyro_raw.y(), gyro_raw.z());
  acc = Vector3(acc.x(), acc.y(), acc.z());
    if(number_of_imu_measurements< 6)
  {
    Rot3 current_rotation = Rot3::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    estimated_rotations_.push_back(current_rotation);
    number_of_imu_measurements++;
  }
  gtsam_graph_->integrateSbgMeasurement(acc, sbg_gyro, gtsam_graph_->getSbgRate());
}


void StateEstimator::dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg)
{  
    Vector3 vel_dvl(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    latest_dvl_measurement_ = vel_dvl;
    covariance_dvl_ << 
          msg->velocity_covariance[0],  
          msg->velocity_covariance[4],  
          msg->velocity_covariance[8];  
    dvl_gyro = gyro;
    new_dvl_measurement_ = true;
}



void StateEstimator::barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  double measured_pressure = msg->fluid_pressure;
  double water_density  = gtsam_graph_->getWaterDensity();
  double depth = -(measured_pressure - atmospheric_pressure_) / (water_density * 9.818); //Down negative 
  // RCLCPP_INFO(this->get_logger(), "Barometer  depth: %f", depth);
  // geometry_msgs::msg::PoseStamped depth_msg;
  // depth_msg.header.stamp = msg->header.stamp;
  // depth_msg.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::PRESS_LINK; //
  // depth_msg.pose.position.x = 0.0; 
  // depth_msg.pose.position.y = 0.0; 
  // depth_msg.pose.position.z = depth; 
  // depth_pub_->publish(depth_msg);
  if(map_initialized_ && is_graph_initialized_){
    if (!baro_calibrated) {
      auto ext = gtsam_graph_->getExtrinsics();
      gtsam::Vector3 base_to_pressure_offset = ext.baro_sensor_offset;
      gtsam::Vector3 sensor_offset = previous_state_.rotation().rotate(base_to_pressure_offset);
      static_offset_ =  depth-sensor_offset.z(); // this is the offset to the static frame
      baro_calibrated = true;
    }
  latest_depth_measurement_ =  depth - static_offset_; // depth in the odom frame
  new_barometer_measurement_received_ = true;
  }
//   try{
// // //     //lookup the depth of pressure in the mocap frame
//     geometry_msgs::msg::TransformStamped T_mocap_from_pressure = tf_buffer_.lookupTransform(
//       "mocap", 
//       name_space_ + "/" + sam_msgs::msg::Links::PRESS_LINK,
//       msg->header.stamp,       
//       tf2::durationFromSec(0.1) 
//     );
//     geometry_msgs::msg::PoseWithCovarianceStamped pressure_pose;
//     pressure_pose.header.stamp = msg->header.stamp;
//     pressure_pose.header.frame_id = "mocap"; 
//     pressure_pose.pose.pose.position.x = T_mocap_from_pressure.transform.translation.x;
//     pressure_pose.pose.pose.position.y = T_mocap_from_pressure.transform.translation.y;
//     pressure_pose.pose.pose.position.z = T_mocap_from_pressure.transform.translation.z;
//   //  Publish the pressure poses
//     gt_pressure_pub_->publish(pressure_pose);
  
//   } catch (tf2::TransformException &ex) {
//     RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
//     return;
//   }
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
  if(this->get_parameter("use_sim_time").as_bool())
  {
    try
    {
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
  else
   {
    RCLCPP_INFO(this->get_logger(), "Waiting for GPS fix to initialize map frame");
    double var = msg->position_covariance[0];
    if (var > cov_threshold_*cov_threshold_)
    {
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
      map_transform.header.frame_id = "utm_" + std::to_string(utm_zone) + "_V"; //need to get the correct band somehow
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
  int utm_zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, utm_zone, northp, utm_x, utm_y);
  utm_z = msg->altitude;
  // Compare the new gps message with the first one to get the offset, but we need it in the odom frame
  if(is_graph_initialized_){
    Point3 map_to_odom_offset;
    Rot3 map_to_odom_rotation;
    try{
      transformStamped = tf_buffer_.lookupTransform("map", name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK,
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
    Point3 gps_in_map(utm_x - first_utm_x, utm_y - first_utm_y, utm_z - first_utm_z);
    // Apply rotation from map to odom
    Point3 gps_in_odom = map_to_odom_rotation.inverse().rotate(gps_in_map - map_to_odom_offset);
    latest_gps_point_ = gps_in_odom;
    position_variances << 
        msg->position_covariance[0],  
        msg->position_covariance[4],  
        msg->position_covariance[8];  
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





void StateEstimator::KeyframeTimerCallback()
{
  // need to have at least 6 imu measurements to initialize the graph with the current orientation
  // auto t1 = std::chrono::high_resolution_clock::now();
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
        //look up the base link to odom transform
        geometry_msgs::msg::TransformStamped odom_transform;
        try {
          odom_transform = tf_buffer_.lookupTransform(
            name_space_+"/"+sam_msgs::msg::Links::ODOM_LINK, "sam_mocap2/base_link",
            tf2::TimePointZero, std::chrono::seconds(1));
        } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
          return;
        }
        tf2::Quaternion q;
        tf2::fromMsg(odom_transform.transform.rotation, q);
        tf2::Quaternion q_ned_to_enu; 
        q_ned_to_enu.setRPY(M_PI, 0.0, 0.0);     
        tf2::Quaternion q_enu =  q * q_ned_to_enu ;
        q_enu.normalize();
        //Extrect the orientation from the transform

        initial_quat = gtsam::Quaternion(q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
      }
      else
      {
        Rot3 average_rotation = averageRotations(estimated_rotations_);
        Quaternion quat = average_rotation.toQuaternion();
        // Initialize the odom frame from map
        auto ext = gtsam_graph_->getExtrinsics();
        gtsam::Vector3 base_to_gps_offset = ext.gps_sensor_offset;
        tf2::Quaternion q_tf2( quat.x(), quat.y(), quat.z(), quat.w());

        tf2::Vector3 off_base(base_to_gps_offset.x(),base_to_gps_offset.y(),base_to_gps_offset.z()  );
        tf2::Vector3 off_map = tf2::quatRotate(q_tf2, off_base);
        geometry_msgs::msg::TransformStamped odom_transform;
        odom_transform.header.stamp = this->get_clock()->now();
        odom_transform.header.frame_id = "map";
        odom_transform.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK;
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
      }
      
      Point3 initial_position = Point3(0.0, 0.0, 0.0);
      // Broadcast the initial pose.
      geometry_msgs::msg::TransformStamped init_transform;
      init_transform.header.stamp = this->get_clock()->now();
      init_transform.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK; 
      init_transform.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK; 
      init_transform.transform.translation.x = initial_position.x();
      init_transform.transform.translation.y = initial_position.y();
      init_transform.transform.translation.z = initial_position.z();
      
      init_transform.transform.rotation.x = initial_quat.x();
      init_transform.transform.rotation.y = initial_quat.y();
      init_transform.transform.rotation.z = initial_quat.z();
      init_transform.transform.rotation.w = initial_quat.w();
      tf_broadcast_.sendTransform(init_transform);
      Vector3 initial_velocity = Vector3(init_vel_odom_.velocity.linear.x,
                                         init_vel_odom_.velocity.linear.y,
                                         init_vel_odom_.velocity.linear.z);
      initial_velocity = Vector3(0,0,0);
      // Initialize the GTSAM graph and state.static_offset_
      gtsam_graph_->initGraphAndState(initial_quat, initial_position,initial_velocity);
      RCLCPP_INFO(this->get_logger(),"initial velocity: [%f, %f, %f]",
                  initial_velocity.x(), initial_velocity.y(), initial_velocity.z());
      RCLCPP_INFO(this->get_logger(),"Graph initialized !!!");
      previous_state_ = gtsam_graph_->getCurrentState();
      is_graph_initialized_ = true;
      current_time = this->get_clock()->now().seconds();
      last_time_ = current_time;
      return;
    
    }
  
  auto [imu_dt, sbg_dt] = gtsam_graph_->getTij();
  if (imu_dt <= 0.0 || sbg_dt <= 0.0)
  {
    RCLCPP_DEBUG(get_logger(),"No new IMU/SBG data this cycle (imu_dt=%.6f, sbg_dt=%.6f), skipping factors + optimize",
      imu_dt, sbg_dt);
    return;
  }

  if(using_motion_model_)
  {
    double current_time = this->get_clock()->now().seconds();
    NavState state = NavState(previous_state_.pose(), previous_state_.velocity());

    NavState new_state = pmm->predict(state, gyro, last_time_, current_time);
    gtsam_graph_->addMotionModelFactor(last_time_,current_time,pmm,gyro,new_state);
    last_time_ = current_time;
    nav_msgs::msg::Odometry motion_model_odom;
    motion_model_odom.header.stamp = this->get_clock()->now();
    motion_model_odom.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK;
    motion_model_odom.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK;

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
  NavState predictes_imu_state = gtsam_graph_->addImuFactor();

  NavState predicted_sbg_state = gtsam_graph_->addSbgFactor();
  // RCLCPP_INFO(this->get_logger(), "SBG prediction state: [%f, %f, %f]",


  if(init_from_ground_truth_) 
  {
    gtsam_graph_->addGtVelocityFactor(gt_velocity_);
  }
  // Add the DVL, GPS and Barometer factors to the graph.
  if (new_dvl_measurement_) {  
    gtsam_graph_->addDvlFactor(latest_dvl_measurement_, dvl_gyro, covariance_dvl_, use_sensor_covariance_);
    new_dvl_measurement_ = false;
  }

  if (new_gps_measurement_) {
    gtsam_graph_->addGpsFactor(latest_gps_point_, position_variances, use_sensor_covariance_);
    new_gps_measurement_ = false;
  }

  if (new_barometer_measurement_received_) {
    gtsam_graph_->addBarometerFactor(latest_depth_measurement_);
    new_barometer_measurement_received_ = false;
  }

  gtsam_graph_->optimize();

  if(using_motion_model_){
    pmm->resetIntegration();
  }
  current_imu_bias_ = gtsam_graph_->getCurrentImuBias();
  previous_state_ = gtsam_graph_->getCurrentState();
  
  // Publish the estimated pose.
  nav_msgs::msg::Odometry estimated_pose;
  estimated_pose.header.stamp = this->get_clock()->now();
  estimated_pose.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK; 
  estimated_pose.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK; 
  estimated_pose.pose.pose.position.x = previous_state_.pose().translation().x();
  estimated_pose.pose.pose.position.y = previous_state_.pose().translation().y();
  estimated_pose.pose.pose.position.z = previous_state_.pose().translation().z();
  Quaternion quat = previous_state_.pose().rotation().toQuaternion();
  estimated_pose.pose.pose.orientation.x = quat.x();
  estimated_pose.pose.pose.orientation.y = quat.y();
  estimated_pose.pose.pose.orientation.z = quat.z();
  estimated_pose.pose.pose.orientation.w = quat.w();
  Eigen::Vector3d v_body(
    previous_state_.velocity().x(),
    previous_state_.velocity().y(),
    previous_state_.velocity().z()
  );
  // we are not estimateing the angular vels but the bias so take the current angular from stim
  Eigen::Vector3d w_body(
    gyro.x()-current_imu_bias_.gyroscope().x(),
    gyro.y()-current_imu_bias_.gyroscope().y(),
    gyro.z()-current_imu_bias_.gyroscope().z()
  );

  // rotation matrix from odom to body frame
  Eigen::Matrix3d R = previous_state_.pose().rotation().transpose().matrix();
  Eigen::Vector3d v_odom = R * v_body;
  estimated_pose.twist.twist.linear.x = v_odom.x();
  estimated_pose.twist.twist.linear.y = v_odom.y();
  estimated_pose.twist.twist.linear.z = v_odom.z();
  estimated_pose.twist.twist.angular.x = w_body.x();
  estimated_pose.twist.twist.angular.y = w_body.y();
  estimated_pose.twist.twist.angular.z = w_body.z();
  pose_pub_->publish(estimated_pose);

  // Broadcast estimated pose.
  geometry_msgs::msg::TransformStamped out_transform;
  out_transform.header.stamp = this->get_clock()->now();
  out_transform.header.frame_id = name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK;
  out_transform.child_frame_id = name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK;
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
  // auto t2 = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  // RCLCPP_INFO(get_logger(), "State estimation took %ld ms", duration);




}



int main(int argc, char **argv) {
  py::scoped_interpreter guard{};
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
