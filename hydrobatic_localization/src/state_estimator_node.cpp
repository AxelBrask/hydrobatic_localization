#include "state_estimator_node.h"

StateEstimator::StateEstimator()
    : Node("state_estimator"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      tf_broadcaster_(this)
{
    // Create an instance of GtsamGraph
    gtsam_graph_ = std::make_unique<GtsamGraph>();

    // Setup ROS subscriptions
    stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        sam_msgs::msg::Topics::STIM_IMU_TOPIC, 
        10, 
        std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1)
    );

    // Example if you also want to subscribe to SBG IMU:
    // sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     sam_msgs::msg::Topics::SBG_IMU_TOPIC, 10, 
    //     std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1)
    // );

    dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
        sam_msgs::msg::Topics::DVL_TOPIC, 
        10, 
        std::bind(&StateEstimator::dvlCallback, this, std::placeholders::_1)
    );

    barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        sam_msgs::msg::Topics::DEPTH_TOPIC, 
        10, 
        std::bind(&StateEstimator::barometerCallback, this, std::placeholders::_1)
    );

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        smarc_msgs::msg::Topics::GPS_TOPIC, 
        10, 
        std::bind(&StateEstimator::gpsCallback, this, std::placeholders::_1)
    );

    // Use sim time
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Create keyframe timer
    keyframe_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&StateEstimator::keyframeTimerCallback, this)
    );
}

void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    double current_time_ = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    // If not initialized, accumulate initial orientations
    if(!is_imu_initialized_) {
        number_of_imu_measurements_++;
        gtsam::Rot3 imu_rotation = gtsam::Rot3::Quaternion(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z
        );
        estimated_rotations_.push_back(imu_rotation);
        last_time_ = current_time_;
        return;
    }

    // Otherwise, integrate IMU measurements
    // double delta_t = 0.01; // Example fixed timestep
    // last_time_ = current_time;
    double delta_t = (last_time_ > 0.0) ? (current_time_ - last_time_) : 1.0/50.0;
    last_time_ = current_time_;
        if (delta_t <= 0) {

        last_time_ = current_time_;
        delta_t = 1.0/50.0;
        }
    gtsam::Vector3 acc(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );

    gtsam::Vector3 gyro(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );

    // Integrate with the GTSAM graph
    gtsam_graph_->integrateIMUMeasurement(acc, gyro, delta_t);
}

void StateEstimator::dvlCallback(const smarc_msgs::msg::DVL::SharedPtr msg)
{
    gtsam::Vector3 vel_dvl(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    latest_dvl_measurement_ = vel_dvl;
    new_dvl_measurement_ = true;
    // RCLCPP_INFO(this->get_logger(), "DVL velocity: [%f, %f, %f]",
    //             vel_dvl.x(), vel_dvl.y(), vel_dvl.z());
}

void StateEstimator::barometerCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
}

void StateEstimator::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
}

void StateEstimator::keyframeTimerCallback()
{
    // If not initialized, check if we have enough IMU samples
    if(!is_imu_initialized_) {
        if(number_of_imu_measurements_ > 5) {
            // Compute average orientation
            gtsam::Rot3 initial_orientation = 
                GtsamGraph::averageRotations(estimated_rotations_);

            // Attempt to lookup transform to get initial position
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                // Example frames: "sam_auv_v1/odom_gt" and "sam_auv_v1/base_link_gt"
                transform_stamped = tf_buffer_.lookupTransform(
                    "sam_auv_v1/odom_gt", 
                    "sam_auv_v1/base_link_gt",
                    tf2::TimePointZero,
                    std::chrono::seconds(1)
                );

                initial_position_ = gtsam::Point3(
                    transform_stamped.transform.translation.x,
                    transform_stamped.transform.translation.y,
                    transform_stamped.transform.translation.z
                );

                initial_rotation_ = gtsam::Rot3(
                    transform_stamped.transform.rotation.w,
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z
                );

                RCLCPP_INFO(this->get_logger(), 
                    "Initial position: [%f, %f, %f]",
                    initial_position_.x(),
                    initial_position_.y(),
                    initial_position_.z()
                );
            } 
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), 
                    "Could not get transform for init: %s", ex.what());
                return;
            }

            // Initialize the GTSAM graph with these
            gtsam_graph_->initializeGraph(initial_rotation_, initial_position_);

            // Mark that we are done with IMU initialization
            is_imu_initialized_ = true;

            // Publish an initial transform for visualization (optional)
            geometry_msgs::msg::TransformStamped init_tf;
            init_tf.header.stamp = this->now();
            init_tf.header.frame_id = "sam_auv_v1/odom_gt";
            init_tf.child_frame_id  = "estimated_pose";
            init_tf.transform.translation.x = initial_position_.x();
            init_tf.transform.translation.y = initial_position_.y();
            init_tf.transform.translation.z = initial_position_.z();

            gtsam::Quaternion quat = initial_rotation_.toQuaternion();
            init_tf.transform.rotation.x = quat.x();
            init_tf.transform.rotation.y = quat.y();
            init_tf.transform.rotation.z = quat.z();
            init_tf.transform.rotation.w = quat.w();

            tf_broadcaster_.sendTransform(init_tf);
        }
        return;
    }

    // Once initialized, call GTSAM keyframe update
    gtsam_graph_->keyframeUpdate(new_dvl_measurement_, latest_dvl_measurement_);
    new_dvl_measurement_ = false; // reset

    // Broadcast the updated pose
    geometry_msgs::msg::TransformStamped estimated_tf;
    estimated_tf.header.stamp = this->now();
    estimated_tf.header.frame_id = "sam_auv_v1/odom_gt";
    estimated_tf.child_frame_id  = "estimated_pose";

    gtsam::Pose3 current_pose = gtsam_graph_->getCurrentPose();
    estimated_tf.transform.translation.x = current_pose.translation().x();
    estimated_tf.transform.translation.y = current_pose.translation().y();
    estimated_tf.transform.translation.z = current_pose.translation().z();

    gtsam::Quaternion gtsam_quat = current_pose.rotation().toQuaternion();
    estimated_tf.transform.rotation.x = gtsam_quat.x();
    estimated_tf.transform.rotation.y = gtsam_quat.y();
    estimated_tf.transform.rotation.z = gtsam_quat.z();
    estimated_tf.transform.rotation.w = gtsam_quat.w();

    tf_broadcaster_.sendTransform(estimated_tf);
}

gtsam::Rot3 StateEstimator::averageRotations(const std::vector<gtsam::Rot3>& rotations)
{
    // Simple wrapper if you wanted it local to the node
    // But you can also just call GtsamGraph::averageRotations.
    return GtsamGraph::averageRotations(rotations);
}

// ------------------- main -------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
