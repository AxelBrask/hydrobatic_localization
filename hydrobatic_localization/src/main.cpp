//ROS Includes
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


//GTSAM Includes
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>    
#include <gtsam/inference/Symbol.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>
#include <boost/optional.hpp>
#include <gtsam/navigation/GPSFactor.h>
#include <hydrobatic_localization/BarometerFactor.h>
#include <hydrobatic_localization/DvlFactor.h>
// geographic lib
#include <GeographicLib/LocalCartesian.hpp>
#include <vector>
using namespace gtsam;
using symbol_shorthand::X; // Pose
using symbol_shorthand::V; // Velocity
using symbol_shorthand::B; // Bias










class StateEstimator : public rclcpp::Node {
public:
    StateEstimator() : Node("state_estimator"),tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),tf_broadcast_(this), current_index_(0) {
        stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            sam_msgs::msg::Topics::STIM_IMU_TOPIC, 10, std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));
        // sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     sam_msgs::msg::Topics::SBG_IMU_TOPIC, 10, std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));    
        dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
            sam_msgs::msg::Topics::DVL_TOPIC,  10, std::bind(&StateEstimator::dvl_callback, this, std::placeholders::_1));
        // barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        //     sam_msgs::msg::Topics::DEPTH_TOPIC, 10, std::bind(&StateEstimator::barometer_callback, this, std::placeholders::_1));
        // gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        //     smarc_msgs::msg::Topics::GPS_TOPIC, 10, std::bind(&StateEstimator::gps_callback, this, std::placeholders::_1));
        
        //use sim time
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        // gtsam::ISAM2Params parameters;
        // isam_ = std::make_shared<gtsam::ISAM2>(parameters);

        KeyframeTimer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&StateEstimator::KeyframeTimerCallback, this));



        // Publishers
        est_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("estimated_odometry", 10);
        dvl_pub_ = this->create_publisher<smarc_msgs::msg::DVL>("fixed_dvl", 10);
        imu_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("imu_vel_estimate", 10);
        auto imu_param = imu_params();
        imuBias::ConstantBias prior_bias;
        imu_preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_param,prior_bias);
    }

private:
    // GTSAM Components
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    // std::shared_ptr<gtsam::ISAM2> isam_;
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrated_;
    // ROS Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr est_odometry_pub_;    

    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr stim_imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sbg_imu_sub_;
    rclcpp::Subscription<smarc_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    //test publisher for dvl
    rclcpp::Publisher<smarc_msgs::msg::DVL>::SharedPtr dvl_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_odom_pub;
    //publish point and the pose
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub_;
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped transformStamped;
    tf2_ros::TransformBroadcaster tf_broadcast_;

    rclcpp::TimerBase::SharedPtr KeyframeTimer;
    int number_of_imu_measurements = 0;
    int current_index_;
    gtsam::NavState current_state_;
    gtsam::imuBias::ConstantBias current_bias_;
    //estimated inital rotation
    std::vector<gtsam::Rot3> estimated_rotations_;
    gtsam::Point3 initial_position;
    gtsam::Rot3 initial_rotation;
    gtsam::NavState previous_state_;
    bool is_imu_initialized_ = false;
    double current_time;
    double last_time_;
    // dvl
    gtsam::Vector3 latest_dvl_measurement_;
    bool new_dvl_measurement_ = false;
    bool new_gps_measurement_ = false;
    gtsam::Vector3 latest_gps_point_;
    gtsam::Vector3 gyro;
    bool converter_initialized_ = false;
    std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
    //Barometer
    double first_barometer_measurement_;
    bool new_barometer_measurement_received_ = false;
    double atmospheric_pressure_ = 101325.0;
    double latest_depth_measurement_;


    // barometer
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if(!is_imu_initialized_){
            number_of_imu_measurements++;
            gtsam::Rot3 imu_rotation = Rot3::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            estimated_rotations_.push_back(imu_rotation);
            last_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            return;
        }
        current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        number_of_imu_measurements++;

        gtsam::Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        gtsam::Vector3 gyro_raw(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        gyro = gtsam::Vector3(-gyro_raw.x(), -gyro_raw.y(), -gyro_raw.z());
        
        // double delta_t =0.1;
        double delta_t =1.0/100.0;
        //  current_time - last_time_;
        // RCLCPP_INFO(this->get_logger(), "Delta T: %f", delta_t); 
        imu_preintegrated_->integrateMeasurement(acc, gyro, delta_t);
        last_time_ = current_time;
        
        

    }
    
    void dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg) {

         gtsam::Vector3 vel_dvl(
            msg->velocity.x, msg->velocity.y, msg->velocity.z);
    // Transform DVL velocity to base_link frame
    try {
        geometry_msgs::msg::TransformStamped dvl_to_baselink_tf =
            tf_buffer_.lookupTransform("sam_auv_v1/base_link_gt", "sam_auv_v1/dvl_link_gt", tf2::TimePointZero);

        gtsam::Rot3 dvl_to_baselink_rot = gtsam::Rot3::Quaternion(
            dvl_to_baselink_tf.transform.rotation.w,
            dvl_to_baselink_tf.transform.rotation.x,
            dvl_to_baselink_tf.transform.rotation.y,
            dvl_to_baselink_tf.transform.rotation.z
        );

        gtsam::Vector3 r_dvl(
            dvl_to_baselink_tf.transform.translation.x,
            dvl_to_baselink_tf.transform.translation.y,
            dvl_to_baselink_tf.transform.translation.z
        );
        
 
        
        gtsam::Vector3 vel_offset = gyro.cross(r_dvl);
        
        gtsam::Vector3 vel_baselink = dvl_to_baselink_rot * vel_dvl - vel_offset;
        
        latest_dvl_measurement_ = vel_dvl;
        new_dvl_measurement_ = true;

        // RCLCPP_INFO(this->get_logger(), "DVL Velocity in Base Link: [%f, %f, %f]", 
        //             vel_baselink.x(), vel_baselink.y(), vel_baselink.z());
        
        // Publish transformed velocity
        smarc_msgs::msg::DVL transformed_dvl_msg;
        transformed_dvl_msg.header.stamp = this->get_clock()->now();
        transformed_dvl_msg.velocity.x = vel_baselink.x();
        transformed_dvl_msg.velocity.y = vel_baselink.y();
        transformed_dvl_msg.velocity.z = vel_baselink.z();
        dvl_pub_->publish(transformed_dvl_msg);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform DVL velocity: %s", ex.what());
    }
    }
    
    void barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {

        if(!is_imu_initialized_ || first_barometer_measurement_ == 0.0){
            first_barometer_measurement_ = msg->fluid_pressure;
            return;
        }
        double measured_pressure = msg->fluid_pressure;
        double depth = (measured_pressure - atmospheric_pressure_)/9806.65; //values taken from dr 
        gtsam::Vector3 base_to_pressure_offset(-0.503, 0.025, 0.057);
        // rotate the depth to base_link frame with the previouse pose
        gtsam::Rot3 R = previous_state_.pose().rotation();
        gtsam::Vector3 rotated_offset = R.rotate(base_to_pressure_offset);

        // double depth_base_link = -depth - 0.057;
        latest_depth_measurement_ = -depth - rotated_offset.z();    
        RCLCPP_INFO(this->get_logger(), "Depth: %f", latest_depth_measurement_);

        new_barometer_measurement_received_ = true;




    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        
            if(!converter_initialized_){
           GeographicLib::LocalCartesian temp_cart(msg->latitude, msg->longitude, msg->altitude);

            // Invert the URDF offset to figure out lat/lon/alt for base_link
            double lat_bl, lon_bl, alt_bl;
            temp_cart.Reverse(-0.528, 0.0, -0.071, lat_bl, lon_bl, alt_bl);

            // 2) Initialize local_cartesian_ so that base_link is (0,0,0)
            local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(lat_bl, lon_bl, alt_bl);
            converter_initialized_ = true;
            }

    // Convert the GPS to local cartesian
    double x, y, z;

    //set frame to the intial position
    local_cartesian_->Forward(msg->latitude, msg->longitude, msg->altitude, x,y,z);
    // RCLCPP_INFO(this->get_logger(), "GPS point in base_link: [%f, %f, %f]", x, y, z);



    // Convert ENU to NED for GTSAM compatibility
    double x_NED = x;
    double y_NED = y;
    double z_NED = z;
    latest_gps_point_ = gtsam::Point3(x_NED, y_NED, z_NED);
    // RCLCPP_INFO(this->get_logger(), "GPS point: [%f, %f, %f]", x_NED, y_NED, z_NED);
    new_gps_measurement_ = true;
    }

    gtsam::Rot3 averageRotations(const std::vector<gtsam::Rot3>& rotations) {
        gtsam::Vector3 sumLog = gtsam::Vector3::Zero();
        for (const auto& rot : rotations) {
            sumLog += gtsam::Rot3::Logmap(rot);
        }
        gtsam::Vector3 avgLog = sumLog / static_cast<double>(rotations.size());
        return gtsam::Rot3::Expmap(avgLog);
    }

    void KeyframeTimerCallback() {
        if(!is_imu_initialized_){
            if(number_of_imu_measurements >5){
                gtsam::Rot3 inital_orientation = averageRotations(estimated_rotations_);
                //getting the intial postion of baselink in the map frame
                try {
                transformStamped = tf_buffer_.lookupTransform("sam_auv_v1/odom_gt", "sam_auv_v1/base_link_gt", tf2::TimePointZero, std::chrono::seconds(1));
                initial_position = gtsam::Point3(transformStamped.transform.translation.x,transformStamped.transform.translation.y,transformStamped.transform.translation.z);
                initial_rotation = gtsam::Rot3(transformStamped.transform.rotation.w,transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z);
                RCLCPP_INFO(this->get_logger(), "Initial Position: [%f, %f, %f]", 
                    initial_position.x(),
                    initial_position.y(),
                    initial_position.z());
                } 
                catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
                    return;
                }


                // Broadcassting inital pose
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "Time: %f", transformStamped.header.stamp.sec + transformStamped.header.stamp.nanosec * 1e-9);
                transformStamped.header.frame_id = "sam_auv_v1/odom_gt";  
                transformStamped.child_frame_id = "estimated_pose"; 
                // TODO:check NED convention of sensore
                transformStamped.transform.translation.x = initial_position.x();
                transformStamped.transform.translation.y = initial_position.y();
                transformStamped.transform.translation.z = initial_position.z();

                gtsam::Quaternion quat = initial_rotation.toQuaternion();
                transformStamped.transform.rotation.x = quat.x();
                transformStamped.transform.rotation.y = quat.y();
                transformStamped.transform.rotation.z = quat.z();
                transformStamped.transform.rotation.w = quat.w();

                tf_broadcast_.sendTransform(transformStamped);



                initGraphAndState(initial_rotation,initial_position);
                is_imu_initialized_ = true;
                }
                return;
            }

        
        int next_index = current_index_ + 1;
        PreintegratedCombinedMeasurements pim = *imu_preintegrated_;
        CombinedImuFactor imu_factor(
            X(current_index_), V(current_index_),
            X(next_index),     V(next_index),
            B(current_index_), B(next_index),
            pim
        );
        graph_.add(imu_factor);

        gtsam::NavState predicted_state = imu_preintegrated_->predict(previous_state_, current_bias_);
        RCLCPP_INFO(this->get_logger(), "Predicted IMU State: [%f, %f, %f]",
                    predicted_state.pose().translation().x(),
                    predicted_state.pose().translation().y(),
                    predicted_state.pose().translation().z());


        //
        initial_estimate_.insert(X(next_index), predicted_state.pose());
        initial_estimate_.insert(V(next_index), predicted_state.v());
        initial_estimate_.insert(B(next_index), current_bias_);
        //publish the imu velocity
        nav_msgs::msg::Odometry imu_odom_msg;
        imu_odom_msg.header.stamp = this->get_clock()->now();
        imu_odom_msg.header.frame_id = "sam_auv_v1/odom_gt";
        imu_odom_msg.child_frame_id = "sam_auv_v1/base_link_gt";
        imu_odom_msg.pose.pose.position.x = predicted_state.pose().translation().x();
        imu_odom_msg.pose.pose.position.y = predicted_state.pose().translation().y();
        imu_odom_msg.pose.pose.position.z = predicted_state.pose().translation().z();
        gtsam::Quaternion quat_imu = predicted_state.pose().rotation().toQuaternion();
        imu_odom_msg.pose.pose.orientation.x = quat_imu.x();
        imu_odom_msg.pose.pose.orientation.y = quat_imu.y();
        imu_odom_msg.pose.pose.orientation.z = quat_imu.z();
        imu_odom_msg.pose.pose.orientation.w = quat_imu.w();
        imu_odom_msg.twist.twist.linear.x = predicted_state.v().x();
        imu_odom_msg.twist.twist.linear.y = predicted_state.v().y();
        imu_odom_msg.twist.twist.linear.z = predicted_state.v().z();
        imu_odom_pub->publish(imu_odom_msg);


        // gtsam::Vector3 new_dvl_velocity = latest_dvl_measurement_;
        if(new_dvl_measurement_){
            new_dvl_measurement_ = false;
            // auto dvl_noise = noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.01, 0.01, 0.1).finished());

            // gtsam::Vector3 dvl_velocity = previous_state_.rotation()*latest_dvl_measurement_;
            // graph_.add(PriorFactor<Vector3>(V(next_index), dvl_velocity, dvl_noise));
            //         // RCLCPP_INFO(this->get_logger(), "DVL Velocity factor added: [%f, %f, %f]", 
            //         // dvl_velocity.x(),
            //         // dvl_velocity.y(),
            //         // dvl_velocity.z());
            auto dvl_noise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.1).finished());
            Vector3 base_link_to_dvl_offset(0.573 ,0.0 ,-0.063); 
            Rot3 base_link_dvl_rotation = Rot3::Identity();
            graph_.add(DvlFactor(X(next_index),V(next_index),B(next_index),
            latest_dvl_measurement_, gyro, base_link_to_dvl_offset, base_link_dvl_rotation, dvl_noise));          
        }
    if(new_gps_measurement_){
        auto gps_noise = noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.5, 0.5, 1).finished());
        // std::cout << "[GtsamGraph] GPS Factor added: " << latest_gps_point_.transpose() << std::endl;
        graph_.add(GPSFactor(X(next_index), latest_gps_point_, gps_noise));
        new_gps_measurement_ = false;

    }

    // if(new_barometer_measurement_received_){
    //     auto barometer_noise = noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.001).finished());
    //     graph_.add(BarometerFactor(X(next_index), latest_depth_measurement_, barometer_noise));
    //     new_barometer_measurement_received_ = false;

    //     }
        // ISAM2Result result = isam_->update(graph_, initial_estimate_);
        LevenbergMarquardtParams params;
        // params.setVerbosity("ERROR");
        // params.setVerbosityLM("SUMMARY");
        LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_, params);
        Values result = optimizer.optimize();
        // result.print("Final Result: ");
        imuBias::ConstantBias latest_bias = result.at<imuBias::ConstantBias>(B(next_index));
        previous_state_ = NavState(result.at<Pose3>(X(next_index)), result.at<Vector3>(V(next_index)));

        // graph_.resize(0);
        // initial_estimate_.clear();

        // Values updated_values = isam_->calculateEstimate();
        // RCLCPP_INFO(this->get_logger(), "Keyframe %d -> Pose: [%f, %f, %f], velocity: [%f, %f, %f]", 
        //             next_index,
        //             previous_state_.pose().translation().x(),
        //             previous_state_.pose().translation().y(),
        //             previous_state_.pose().translation().z(),
        //             previous_state_.v().x(),
        //             previous_state_.v().y(),
        //             previous_state_.v().z());
        

        current_bias_ = latest_bias;
        // RCLCPP_INFO(this->get_logger(), "IMU Bias Before Reset: [%f, %f, %f, %f, %f, %f]", 
        //     current_bias_.accelerometer().x(), 
        //     current_bias_.accelerometer().y(), 
        //     current_bias_.accelerometer().z(),
        //     current_bias_.gyroscope().x(),
        //     current_bias_.gyroscope().y(),
        //     current_bias_.gyroscope().z());
        imu_preintegrated_->resetIntegrationAndSetBias(latest_bias);




        // Publish the estimated odometry
        nav_msgs::msg::Odometry odometry_msg;
        odometry_msg.header.stamp = this->get_clock()->now();
        odometry_msg.header.frame_id = "sam_auv_v1/odom_gt";
        odometry_msg.child_frame_id = "sam_auv_v1/base_link_gt";
        odometry_msg.pose.pose.position.x = previous_state_.pose().translation().x();
        odometry_msg.pose.pose.position.y = previous_state_.pose().translation().y();
        odometry_msg.pose.pose.position.z = previous_state_.pose().translation().z();
        gtsam::Quaternion quat = previous_state_.pose().rotation().toQuaternion();
        odometry_msg.pose.pose.orientation.x = quat.x();
        odometry_msg.pose.pose.orientation.y = quat.y();
        odometry_msg.pose.pose.orientation.z = quat.z();
        odometry_msg.pose.pose.orientation.w = quat.w();
        odometry_msg.twist.twist.linear.x = previous_state_.v().x();
        odometry_msg.twist.twist.linear.y = previous_state_.v().y();
        odometry_msg.twist.twist.linear.z = previous_state_.v().z();
        est_odometry_pub_->publish(odometry_msg);
        current_index_ = next_index;
        geometry_msgs::msg::TransformStamped odom_to_base_gt;
        try {
            odom_to_base_gt = tf_buffer_.lookupTransform("sam_auv_v1/odom_gt", "sam_auv_v1/base_link_gt", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }


        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "sam_auv_v1/odom_gt";      
        transformStamped.child_frame_id  = "estimated_pose"; 

        gtsam::Point3 estimated_translation = previous_state_.pose().translation();
        gtsam::Rot3 estimated_rotation = previous_state_.pose().rotation();

        transformStamped.transform.translation.x = estimated_translation.x();
        transformStamped.transform.translation.y = estimated_translation.y();
        transformStamped.transform.translation.z = estimated_translation.z();



        gtsam::Quaternion gtsamQuat = previous_state_.pose().rotation().toQuaternion();
        transformStamped.transform.rotation.x = gtsamQuat.x();
        transformStamped.transform.rotation.y = gtsamQuat.y();
        transformStamped.transform.rotation.z = gtsamQuat.z();
        transformStamped.transform.rotation.w = gtsamQuat.w();

        tf_broadcast_.sendTransform(transformStamped);


                    
    }


    std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_params(){
        double accel_noise_sigma = 2.0e-6;//0.0003924;
        double gyro_noise_sigma = 5.0e-6;//0.000205689024915;
        double accel_bias_rw_sigma = 1e-3;//0.004905;
        double gyro_bias_rw_sigma = 1e-4;//0.000001454441043;
        Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
        Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
        Matrix33 integration_error_cov = I_3x3 * 1e-8;//1e-8;  /
        Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
        Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
        Matrix66 bias_acc_omega_init = I_6x6 * 1e-5;//1e-5;  
        auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
        imu_params->accelerometerCovariance = measured_acc_cov;  
        imu_params->integrationCovariance = integration_error_cov;  
        // should be using 2nd order integration
        // PreintegratedRotation params:
        imu_params->gyroscopeCovariance = measured_omega_cov;  
        // PreintegrationCombinedMeasurements params:
        imu_params->biasAccCovariance = bias_acc_cov;      
        imu_params->biasOmegaCovariance = bias_omega_cov;  
        imu_params->biasAccOmegaInt = bias_acc_omega_init;
        imu_params->body_P_sensor = Pose3(
        Rot3(),    
        Point3(0.24, 0.0, -0.036));
        return imu_params;
        
    };

        void initGraphAndState(const gtsam::Rot3& initial_rot,const gtsam::Point3& initial_position){
        RCLCPP_INFO(this->get_logger(), "Initializing the graph and state, initial position: [%f, %f, %f]", 
                    initial_position.x(),
                    initial_position.y(),
                    initial_position.z());
        Pose3 prior_pose = Pose3(initial_rot, initial_position);
        Vector3 prior_velocity(0,0,0);
        imuBias::ConstantBias prior_bias; 

        auto pose_noise = noiseModel::Isotropic::Sigma(6, 1e-3); //as 1e-2
        auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-3);
        auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3);

        
        graph_.addPrior<Pose3>(X(0), prior_pose, pose_noise);
        graph_.addPrior<Vector3>(V(0), prior_velocity, velocity_noise);
        graph_.addPrior<imuBias::ConstantBias>(B(0), prior_bias, bias_noise);

        
        initial_estimate_.insert(X(0), prior_pose);
        initial_estimate_.insert(V(0), prior_velocity);
        initial_estimate_.insert(B(0), prior_bias);

        
        previous_state_ = NavState(prior_pose, prior_velocity);
        current_bias_ = prior_bias;

        
        // isam_->update(graph_, initial_estimate_);
        LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_);


        // graph_.resize(0);
        // initial_estimate_.clear();
        RCLCPP_INFO(this->get_logger(), "Initialized the graph and state");
    }



};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
