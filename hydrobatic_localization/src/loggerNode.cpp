#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <fstream>

class loggerNode : public rclcpp::Node {

public:
    loggerNode() : Node("logger_node"), tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
            geometry_msgs::msg::TransformStamped odom_to_odom_gt;
   
        get_static_utm_map_gt();
        log_file_.open("state_estimator_log.csv");
        log_file_ << "time, est_pos_x, est_pos_y, est_pos_z, est_quat_w, est_quat_x, est_quat_y, est_quat_z, "
                  << "gt_pos_x, gt_pos_y, gt_pos_z, gt_quat_w, gt_quat_x, gt_quat_y, gt_quat_z\n";
        // gt_sub_.subscribe(this, "core/odom_gt");
        // est_sub_.subscribe(this, "estimated_pose");
        // sync_ = std::make_shared<message_filters::Synchronizer<sync_policy_>>(sync_policy_(10), gt_sub_, est_sub_);
        // sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.08));
        // sync_->registerCallback(std::bind(&loggerNode::callback, this, std::placeholders::_1, std::placeholders::_2));
        // Timer for periodic logging (50 Hz)
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&loggerNode::logPoses, this));

    }

    ~loggerNode() {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Logger node shutting down");
    }
    void get_static_utm_map_gt()
  {
    try {
      utm_map_gt_ = tf_buffer_.lookupTransform(
        "map",                     // target frame
        "sam_auv_v1/odom_gt", // source frame
        rclcpp::Time(0),           // latest available
        tf2::durationFromSec(2.0)  // timeout
      );
      have_utm_map_gt_ = true;
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
        "Failed to cache UTM→map_gt: %s", ex.what());
        get_static_utm_map_gt();
    }
  }

    // void callback(const nav_msgs::msg::Odometry::ConstSharedPtr gt_msg,
    //               const nav_msgs::msg::Odometry::ConstSharedPtr est_msg) {
    //     // Log the messages
    //     RCLCPP_INFO(this->get_logger(), "Ground Truth: [%f, %f, %f]", 
    //                 gt_msg->pose.pose.position.x, gt_msg->pose.pose.position.y, gt_msg->pose.pose.position.z);
    //     RCLCPP_INFO(this->get_logger(), "Estimated: [%f, %f, %f]", 
    //                 est_msg->pose.pose.position.x, est_msg->pose.pose.position.y, est_msg->pose.pose.position.z);

    //     // They are in different map frames, so we need to convert them to the same frame
    //     log_file_<< gt_msg->header.stamp.sec + gt_msg->header.stamp.nanosec * 1e-9 << ", "
    //              << est_msg->pose.pose.position.x << ", " << est_msg->pose.pose.position.y << ", " << est_msg->pose.pose.position.z<< ", "
    //              << est_msg->pose.pose.orientation.w << ", " << est_msg->pose.pose.orientation.x << ", " 
    //              << est_msg->pose.pose.orientation.y << ", " << est_msg->pose.pose.orientation.z << ", "
    //              << gt_msg->pose.pose.position.x << ", " << gt_msg->pose.pose.position.y << ", " 
    //              << gt_msg->pose.pose.position.z << ", "
    //              << gt_msg->pose.pose.orientation.w << ", " 
    //              << gt_msg->pose.pose.orientation.x << ", " 
    //              << gt_msg->pose.pose.orientation.y << ", " 
    //              << gt_msg->pose.pose.orientation.z
    //              << "\n";
    //     log_file_.flush();


    // }
    void logPoses()
  {
    // Lookup both transforms into 'odom'
    geometry_msgs::msg::TransformStamped tf_est, tf_gt;
    try {

      tf_est = tf_buffer_.lookupTransform(
        "map",           // target frame
        "estimated_pose",// source frame (replace with your estimator frame_id)
        rclcpp::Time(0),   // latest available
        tf2::durationFromSec(0.05));
    tf_gt = tf_buffer_.lookupTransform(
        "sam_auv_v1/odom_gt",           // same target
        "sam_auv_v1/base_link_gt", // source frame (replace with your GT frame_id)
               tf_gt.header.stamp,
        tf2::durationFromSec(0.05));
        tf2::doTransform(tf_gt, tf_gt, utm_map_gt_);

    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    double t = this->now().seconds();
    // Write CSV row: time, est..., gt...
    log_file_ << t << ", "
              << tf_est.transform.translation.x  << ", "
              << tf_est.transform.translation.y  << ", "
              << tf_est.transform.translation.z  << ", "
              << tf_est.transform.rotation.w     << ", "
              << tf_est.transform.rotation.x     << ", "
              << tf_est.transform.rotation.y     << ", "
              << tf_est.transform.rotation.z     << ", "
              << tf_gt.transform.translation.x   << ", "
              << tf_gt.transform.translation.y   << ", "
              << tf_gt.transform.translation.z   << ", "
              << tf_gt.transform.rotation.w      << ", "
              << tf_gt.transform.rotation.x      << ", "
              << tf_gt.transform.rotation.y      << ", "
              << tf_gt.transform.rotation.z      << "\n";
    log_file_.flush();
  }

private:
    // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry> sync_policy_;
    // std::shared_ptr<message_filters::Synchronizer<sync_policy_>> sync_;

    // message_filters::Subscriber<nav_msgs::msg::Odometry> gt_sub_;
    // message_filters::Subscriber<nav_msgs::msg::Odometry> est_sub_;

    std::ofstream log_file_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped utm_map_gt_;
    bool have_utm_map_gt_ = false;
};



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<loggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}