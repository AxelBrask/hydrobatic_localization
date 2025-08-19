#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iomanip>
#include <filesystem>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <fstream>
#include <sam_msgs/msg/links.hpp>

class loggerNode : public rclcpp::Node {

public:
    loggerNode() : Node("logger_node"), tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

        geometry_msgs::msg::TransformStamped odom_to_odom_gt;
        this->declare_parameter<std::string>("folder", "logs");
        this->get_parameter("folder", folder_);
        this->declare_parameter<std::string>("frame_suffix","");
        this->get_parameter("frame_suffix", frame_suffix_);
        std::filesystem::create_directories(folder_);
          name_space_ = this->get_namespace();
        //remove leading slashes from namespace
        if (name_space_.front() == '/') {
          name_space_.erase(0, 1);
        }


        
        log_file_.open(folder_ + "/state_estimator_log.csv");
        log_file_ << "time, est_pos_x, est_pos_y, est_pos_z, est_quat_w, est_quat_x, est_quat_y, est_quat_z, "
                  << "gt_pos_x, gt_pos_y, gt_pos_z, gt_quat_w, gt_quat_x, gt_quat_y, gt_quat_z\n";

        while (!tf_buffer_.canTransform(
           name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK+ (frame_suffix_.empty() ? "" : "_" + frame_suffix_),
           name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK + (frame_suffix_.empty() ? "" : "_" + frame_suffix_),
           tf2::TimePointZero)) {
          rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

        tf_buffer_.canTransform(name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK+ (frame_suffix_.empty() ? "" : "_" + frame_suffix_),
         "sam_mocap/base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));

        tf_buffer_.canTransform(name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK + (frame_suffix_.empty() ? "" : "_" + frame_suffix_),
           name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK + (frame_suffix_.empty() ? "" : "_" + frame_suffix_), tf2::TimePointZero,
            tf2::durationFromSec(1.0));
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&loggerNode::logPoses, this));
        RCLCPP_INFO(this->get_logger(), "Logger node initialized, logging to %s", (folder_ + "/state_estimator_log.csv").c_str());

    }

    ~loggerNode() {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Logger node shutting down");
    }
    

    void logPoses()
    {
    // Lookup both transforms into 'odom'
    geometry_msgs::msg::TransformStamped tf_est, tf_gt;
    rclcpp::Time stamp = this->now();
    try {

      tf_est = tf_buffer_.lookupTransform(
        name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK+ (frame_suffix_.empty() ? "" : "_" + frame_suffix_),           
        name_space_ + "/" + sam_msgs::msg::Links::BASE_LINK + (frame_suffix_.empty() ? "" : "_" + frame_suffix_),
        rclcpp::Time(0),   
        tf2::durationFromSec(0.01));
    tf_gt = tf_buffer_.lookupTransform(
        name_space_ + "/" + sam_msgs::msg::Links::ODOM_LINK+ (frame_suffix_.empty() ? "" : "_" + frame_suffix_),           
        "sam_mocap/base_link", 
        tf_est.header.stamp,
        tf2::durationFromSec(0.01));
        // tf2::doTransform(tf_gt, tf_gt, utm_map_gt_);

    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }
    rclcpp::Time now_stamp = tf_est.header.stamp;
    if (now_stamp <= last_stamp_) {
      // same or older than what we logged last time → skip
      return;
    }
    last_stamp_ = now_stamp;

    tf2::Quaternion q_est(
      tf_est.transform.rotation.x,
      tf_est.transform.rotation.y,
      tf_est.transform.rotation.z,
      tf_est.transform.rotation.w );
    q_est.normalize();


    tf2::Quaternion q_gt(
      tf_gt.transform.rotation.x,
      tf_gt.transform.rotation.y,
      tf_gt.transform.rotation.z,
      tf_gt.transform.rotation.w );
    q_gt.normalize();


    tf2::Quaternion q_flip;
    q_flip.setRPY(M_PI, 0.0, 0.0);  
    q_flip.normalize();

    tf2::Quaternion q_gt_corrected = q_gt * q_flip;
    q_gt_corrected.normalize();


    tf2::Quaternion q_yaw90;
    q_yaw90.setRPY(0.0, 0.0, M_PI/2.0);  
    q_yaw90.normalize();

    tf2::Quaternion q_gt_final = q_gt_corrected;
    q_gt_final.normalize();
    const double t = tf_est.header.stamp.sec + tf_est.header.stamp.nanosec * 1e-9;



    log_file_ << std::fixed << std::setprecision(6)
              << t << ", "
              // Estimated pose (already in ENU):
              << tf_est.transform.translation.x  << ", "
              << tf_est.transform.translation.y  << ", "
              << tf_est.transform.translation.z  << ", "
              << tf_est.transform.rotation.w     << ", "
              << tf_est.transform.rotation.x     << ", "
              << tf_est.transform.rotation.y     << ", "
              << tf_est.transform.rotation.z     << ", "
              // GT pose, but *converted* to ENU:
              << tf_gt.transform.translation.x   << ", "  
              << tf_gt.transform.translation.y   << ", "
              << tf_gt.transform.translation.z   << ", "
              << q_gt_final.getW()           << ", "
              << q_gt_final.getX()           << ", "
              << q_gt_final.getY()           << ", "
              << q_gt_final.getZ()           << "\n";
    log_file_.flush();
  }

private:
    std::string folder_;
    std::ofstream log_file_;
    std::string frame_suffix_;
    std::string name_space_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped utm_map_gt_;
    bool have_utm_map_gt_ = false;
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};  
};



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<loggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}