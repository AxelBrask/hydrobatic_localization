#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <fstream>

class loggerNode : public rclcpp::Node {

public:
    loggerNode() : Node("logger_node") {

        log_file_.open("state_estimator_log.csv");
        log_file_ << "time, est_pos_x, est_pos_y, est_pos_z, est_quat_w, est_quat_x, est_quat_y, est_quat_z, "
                  << "gt_pos_x, gt_pos_y, gt_pos_z, gt_quat_w, gt_quat_x, gt_quat_y, gt_quat_z\n";
        gt_sub_.subscribe(this, "core/odom_gt");
        est_sub_.subscribe(this, "estimated_pose");
        sync_ = std::make_shared<message_filters::Synchronizer<sync_policy_>>(sync_policy_(10), gt_sub_, est_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.04));
        sync_->registerCallback(std::bind(&loggerNode::callback, this, std::placeholders::_1, std::placeholders::_2));


    }

    ~loggerNode() {
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Logger node shutting down");
    }

    void callback(const nav_msgs::msg::Odometry::ConstSharedPtr gt_msg,
                  const nav_msgs::msg::Odometry::ConstSharedPtr est_msg) {
        // Log the messages
        RCLCPP_INFO(this->get_logger(), "Ground Truth: [%f, %f, %f]", 
                    gt_msg->pose.pose.position.x, gt_msg->pose.pose.position.y, gt_msg->pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Estimated: [%f, %f, %f]", 
                    est_msg->pose.pose.position.x, est_msg->pose.pose.position.y, est_msg->pose.pose.position.z);

        // They are in different map frames, so we need to convert them to the same frame
        log_file_<< gt_msg->header.stamp.sec + gt_msg->header.stamp.nanosec * 1e-9 << ", "
                 << est_msg->pose.pose.position.x << ", " << est_msg->pose.pose.position.y << ", " << est_msg->pose.pose.position.z<< ", "
                 << est_msg->pose.pose.orientation.w << ", " << est_msg->pose.pose.orientation.x << ", " 
                 << est_msg->pose.pose.orientation.y << ", " << est_msg->pose.pose.orientation.z << ", "
                 << gt_msg->pose.pose.position.x << ", " << gt_msg->pose.pose.position.y << ", " 
                 << gt_msg->pose.pose.position.z << ", "
                 << gt_msg->pose.pose.orientation.w << ", " 
                 << gt_msg->pose.pose.orientation.x << ", " 
                 << gt_msg->pose.pose.orientation.y << ", " 
                 << gt_msg->pose.pose.orientation.z
                 << "\n";
        log_file_.flush();


    }

private:
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry> sync_policy_;
    std::shared_ptr<message_filters::Synchronizer<sync_policy_>> sync_;

    message_filters::Subscriber<nav_msgs::msg::Odometry> gt_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> est_sub_;

    std::ofstream log_file_;

};



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<loggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}