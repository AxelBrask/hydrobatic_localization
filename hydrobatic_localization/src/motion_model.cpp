#include <rclcpp/rclcpp.hpp>
#include <hydrobatic_localization/SamMotionModel.h>
// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <smarc_msgs/msg/thruster_feedback.hpp>
#include <smarc_msgs/msg/percent_stamped.hpp>
#include <piml_msgs/msg/thruster_rpm_stamped.hpp> 
#include <sam_msgs/msg/thruster_angles.hpp>
#include <nav_msgs/msg/odometry.hpp>


class MotionModelOnly : public rclcpp::Node {
public:
  MotionModelOnly()
    : Node("motion_model_only"), dt_(0.01),
      has_initial_pose_(false), has_prev_(false), t_prev_(0.0),
      last_lcg_(0.0), last_vbs_(0.0), last_thr1_rpm_(0.0), last_thr2_rpm_(0.0),
      last_vector_vertical_radians_(0.0), last_vector_horizontal_radians_(0.0){
    // Initialize the motion model
    x_.resize(19);
    x_.setZero(); 
    u_prev_.resize(6);  
    u_prev_.setZero();  
    pmm = std::make_shared<SamMotionModelWrapper>(dt_);

    // Message filters for thrusters
    thruster1_sub_.subscribe(this, "piml/thruster1_cmd");
    thruster2_sub_.subscribe(this, "piml/thruster2_cmd");
    thruster_sync_ = std::make_shared<ThrusterSync>(
      ThrusterSyncPolicy(10), thruster1_sub_, thruster2_sub_);
    thruster_sync_->registerCallback(
      std::bind(&MotionModelOnly::thruster_callback, this, std::placeholders::_1, std::placeholders::_2));
    
      // Message filters for LCG and VBS
    lcg_sub_.subscribe(this, "piml/lcg_fb");
    vbs_sub_.subscribe(this, "piml/vbs_fb");
    lcg_vbs_sync_ = std::make_shared<LcgVbsSync>(
      LcgVbsSyncPolicy(10), lcg_sub_, vbs_sub_);
    lcg_vbs_sync_->registerCallback(
      std::bind(&MotionModelOnly::lcg_vbs_callback, this, std::placeholders::_1, std::placeholders::_2));
    
      // Subscription for thruster vector commands
    thruster_vector_sub_ = this->create_subscription<sam_msgs::msg::ThrusterAngles>(
      "piml/thrust_vector_cmd", 10,
      std::bind(&MotionModelOnly::ThrusterVectorCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Motion Model Only Node Initialized");
      gt_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mocap/sam_mocap2/odom", 10,std::bind(&MotionModelOnly::gt_odom_callback, this, std::placeholders::_1));
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("motion_model_odom", 10);
      
  }
    void gt_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
    
    x_(0) = msg->pose.pose.position.x;
    x_(1) = msg->pose.pose.position.y;
    x_(2) = msg->pose.pose.position.z;
    x_(3) = msg->pose.pose.orientation.w;
    x_(4) = msg->pose.pose.orientation.x;
    x_(5) = msg->pose.pose.orientation.y;
    x_(6) = msg->pose.pose.orientation.z;
    x_(7) = msg->twist.twist.linear.x;
    x_(8) = msg->twist.twist.linear.y;
    x_(9) = msg->twist.twist.linear.z;
    x_(10) = msg->twist.twist.angular.x;
    x_(11) = msg->twist.twist.angular.y;
    x_(12) = msg->twist.twist.angular.z;
    
    t_prev_ = msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;
    has_initial_pose_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Initial pose set from ground truth odometry.");
    RCLCPP_INFO(this->get_logger(), "Ground truth pose: x: %f, y: %f, z: %f", 
                x_(0), x_(1), x_(2));
    gt_pose_sub_.reset();     
    }

    void ThrusterVectorCallback(const sam_msgs::msg::ThrusterAngles::SharedPtr msg)
    {
    Eigen::VectorXd u_curr(6);
    last_vector_vertical_radians_ = msg->thruster_vertical_radians;
    last_vector_horizontal_radians_ = msg->thruster_horizontal_radians;
    u_curr << last_lcg_, last_vbs_,
              last_vector_vertical_radians_,
              last_vector_horizontal_radians_,
              last_thr1_rpm_, last_thr2_rpm_;

    

    double t_curr = msg->header.stamp.sec
                    + msg->header.stamp.nanosec*1e-9;
      if(!has_prev_) {
        x_.tail<6>() = u_curr; 
        t_prev_ = t_curr; 
        u_prev_ = u_curr;
        has_prev_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose set from thruster vector callback.");

        return;

    }
    if (has_prev_) {
      double dt = t_curr - t_prev_;
      if (dt > 0.0) {
        x_ = pmm->integrateState(x_, u_prev_, dt);
        publishOdom(x_, t_curr);
      }
    } else {
      has_prev_ = true;
    }

    u_prev_ = u_curr;
    t_prev_ = t_curr;
  }
    // thrusters-only
    void thruster_callback(const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t1,
                            const piml_msgs::msg::ThrusterRPMStamped::ConstSharedPtr t2)
    {

    Eigen::VectorXd u_curr(6);
    last_thr1_rpm_ = t1->rpm;
    last_thr2_rpm_ = t2->rpm;
    u_curr << last_lcg_, last_vbs_,last_vector_vertical_radians_, last_vector_horizontal_radians_, last_thr1_rpm_, last_thr2_rpm_;
    double t_curr = t1->header.stamp.sec
                    + t1->header.stamp.nanosec*1e-9;
    if(!has_prev_) {
      x_.tail<6>() = u_curr; 
      t_prev_ = t_curr; 
      u_prev_ = u_curr;
      has_prev_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial control set from thruster rpm callback.");
      return;

    }

    if (has_prev_) {
      double dt = t_curr - t_prev_;
      if (dt > 0.0) {
        x_ = pmm->integrateState(x_, u_prev_, dt);
        publishOdom(x_, t_curr);
      }
    } else {
      has_prev_ = true;
    }
        // shift in for next interval
    u_prev_ = u_curr; 
    t_prev_ = t_curr;
  }
    // LCG/VBS-only
    void lcg_vbs_callback(
    const smarc_msgs::msg::PercentStamped::ConstSharedPtr lcg,
    const smarc_msgs::msg::PercentStamped::ConstSharedPtr vbs)
    {

    Eigen::VectorXd u_curr(6);
    last_lcg_ = lcg->value;
    last_vbs_ = vbs->value;

    u_curr << last_lcg_, last_vbs_,
              last_vector_vertical_radians_,
              last_vector_horizontal_radians_,
              last_thr1_rpm_, last_thr2_rpm_;
    double t_curr = lcg->header.stamp.sec + lcg->header.stamp.nanosec*1e-9;
    if(!has_prev_) {
      x_.tail<6>() = u_curr; 
      t_prev_ = t_curr; 
      u_prev_ = u_curr;
      has_prev_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial contorl set from lcg/vbs callback.");

      return;

    }


    double dt = t_curr - t_prev_;
    if (dt > 0) {
      x_ = pmm->integrateState(x_, u_prev_, dt);
      t_prev_ = t_curr;
      publishOdom(x_, t_curr);

    }
        // shift in for next interval
    u_prev_ = u_curr;
    t_prev_ = t_curr;
  }
  void publishOdom(const Eigen::VectorXd& x, double t)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(t);
    odom.header.frame_id = "mocap";
    odom.child_frame_id  = "base_link";

    odom.pose.pose.position.x = x(0);
    odom.pose.pose.position.y = x(1);
    odom.pose.pose.position.z = x(2);
    odom.pose.pose.orientation.w = x(3);
    odom.pose.pose.orientation.x = x(4);
    odom.pose.pose.orientation.y = x(5);
    odom.pose.pose.orientation.z = x(6);

    odom.twist.twist.linear.x  = x(7);
    odom.twist.twist.linear.y  = x(8);
    odom.twist.twist.linear.z  = x(9);
    odom.twist.twist.angular.x = x(10);
    odom.twist.twist.angular.y = x(11);
    odom.twist.twist.angular.z = x(12);

    odom_pub_->publish(odom);
  }


private:
  bool has_initial_pose_ = false;
  double dt_;
  Eigen::VectorXd x_;      // 13-element [pos(3), quat(4), vel(3), gyro(3)] or however SamMotionModelWrapper defines it
  Eigen::VectorXd u_prev_;
  double         t_prev_;
  bool           has_prev_;
  std::shared_ptr<SamMotionModelWrapper> pmm;
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

  // All five subscribers
  message_filters::Subscriber<piml_msgs::msg::ThrusterRPMStamped> thruster1_sub_;
  message_filters::Subscriber<piml_msgs::msg::ThrusterRPMStamped> thruster2_sub_;
  message_filters::Subscriber<smarc_msgs::msg::PercentStamped>     lcg_sub_;
  message_filters::Subscriber<smarc_msgs::msg::PercentStamped>     vbs_sub_;
  rclcpp::Subscription<sam_msgs::msg::ThrusterAngles>::SharedPtr thruster_vector_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_pose_sub_;  
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // State variables  
  double last_lcg_ = 0.0; 
  double last_vbs_ = 0.0; 
  double last_thr1_rpm_ = 0.0;    
  double last_thr2_rpm_ = 0.0;    
  double last_vector_vertical_radians_ = 0.0;
  double last_vector_horizontal_radians_ = 0.0;
  // Variable for initial pose and velocity
  

};

int main(int argc, char **argv) {
  py::scoped_interpreter guard{};
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionModelOnly>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}