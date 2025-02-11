#ifndef STATE_ESTIMATOR_NODE_HPP
#define STATE_ESTIMATOR_NODE_HPP

// ROS Includes
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Standard
#include <vector>
#include <memory>

// Forward-declare GtsamGraph class
#include "gtsam_graph.h"

/**
 * @brief The main ROS Node for state estimation, 
 *        which uses a GtsamGraph to handle factor-graph optimization.
 */
class StateEstimator : public rclcpp::Node
{
public:
    StateEstimator();

private:
    // ---------- ROS Callbacks ----------
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void dvlCallback(const smarc_msgs::msg::DVL::SharedPtr msg);
    void barometerCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void keyframeTimerCallback();

    /**
     * @brief Determine initial orientation by averaging a few IMU orientations.
     */
    gtsam::Rot3 averageRotations(const std::vector<gtsam::Rot3>& rotations);

private:
    // ---------- GTSAM Graph Wrapper ----------
    std::unique_ptr<GtsamGraph> gtsam_graph_;

    // ---------- ROS Subscribers ----------
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr stim_imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sbg_imu_sub_;
    rclcpp::Subscription<smarc_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    // ---------- TF ----------
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // ---------- Timers ----------
    rclcpp::TimerBase::SharedPtr keyframe_timer_;

    // ---------- State variables for initialization ----------
    bool is_imu_initialized_ = false;
    int number_of_imu_measurements_ = 0;
    std::vector<gtsam::Rot3> estimated_rotations_;
    double last_time_ = 0.0;

    // ---------- DVL ----------
    gtsam::Vector3 latest_dvl_measurement_ = gtsam::Vector3::Zero();
    bool new_dvl_measurement_ = false;

    // ---------- For storing initial pose from TF ----------
    gtsam::Point3 initial_position_;
    gtsam::Rot3   initial_rotation_;

};

#endif // STATE_ESTIMATOR_NODE_HPP
