//ROS Includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sam_msgs/msg/topics.hpp>
#include <smarc_msgs/msg/topics.hpp>
#include <smarc_msgs/msg/dvl.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
//GTSAM Includes
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>    
#include <gtsam/inference/Symbol.h>
using namespace gtsam;
using symbol_shorthand::X; // Pose
using symbol_shorthand::V; // Velocity
using symbol_shorthand::B; // Bias
class StateEstimator : public rclcpp::Node {
public:
    StateEstimator() : Node("state_estimator"), last_imu_time_(0.0), current_index_(0) {
        stim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            sam_msgs::msg::Topics::STIM_IMU_TOPIC, 10, std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));
        // sbg_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     sam_msgs::msg::Topics::SBG_IMU_TOPIC, 10, std::bind(&StateEstimator::imu_callback, this, std::placeholders::_1));    
        dvl_sub_ = this->create_subscription<smarc_msgs::msg::DVL>(
            sam_msgs::msg::Topics::DVL_TOPIC, 10, std::bind(&StateEstimator::dvl_callback, this, std::placeholders::_1));
        barometer_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            sam_msgs::msg::Topics::DEPTH_TOPIC, 10, std::bind(&StateEstimator::barometer_callback, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            smarc_msgs::msg::Topics::GPS_TOPIC, 10, std::bind(&StateEstimator::gps_callback, this, std::placeholders::_1));
        

        gtsam::ISAM2Params parameters;
        isam_ = std::make_shared<gtsam::ISAM2>(parameters);

        KeyframeTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&StateEstimator::KeyframeTimerCallback, this));

        initGraphAndState();

        auto imu_param = imu_params();
        imuBias::ConstantBias prior_bias;
        imu_preintegrated_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_param,prior_bias);
    }

private:
    // GTSAM Components
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    std::shared_ptr<gtsam::ISAM2> isam_;
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrated_;
    
    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr stim_imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sbg_imu_sub_;
    rclcpp::Subscription<smarc_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    rclcpp::TimerBase::SharedPtr KeyframeTimer;

    double last_imu_time_;
    int current_index_;
    gtsam::NavState current_state_;
    gtsam::imuBias::ConstantBias current_bias_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {

        double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        gtsam::Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        gtsam::Vector3 gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        if (last_imu_time_ < 0.0) {
        // This is the first measurement; set the time and skip
        last_imu_time_ = current_time;
        return;
        }
        // Otherwise compute delta
        double delta_t = current_time - last_imu_time_;
        last_imu_time_ = current_time;
        imu_preintegrated_->integrateMeasurement(acc, gyro, delta_t);
        
        

    }
    
    void dvl_callback(const smarc_msgs::msg::DVL::SharedPtr msg) {
    }
    
    void barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    }



    void KeyframeTimerCallback() {
        if(last_imu_time_ < 0.0) {
            return;
        }
        int next_index = current_index_ + 1;
        RCLCPP_INFO(this->get_logger(),"in keyframe timer callback");
     PreintegratedCombinedMeasurements pim = *imu_preintegrated_;
        CombinedImuFactor imu_factor(
            X(current_index_), V(current_index_),
            X(next_index),     V(next_index),
            B(current_index_), B(next_index),
            pim
        );
        graph_.add(imu_factor);

        // 2. We predict forward from the *current best estimate* using the preintegration
        gtsam::NavState predicted_state = imu_preintegrated_->predict(current_state_, current_bias_);

        // 3. Insert initial guesses for X(next), V(next), B(next)
        //    a) We use the integrated/predicted state for (X(next), V(next))
        //    b) We often just re-use the old bias as a guess for B(next)
        initial_estimate_.insert(X(next_index), predicted_state.pose());
        initial_estimate_.insert(V(next_index), predicted_state.v());
        initial_estimate_.insert(B(next_index), current_bias_);

        // (OPTIONAL) 4. If you have GPS or other measurements at this time,
        //    add them to the factor graph here, e.g.:
        //
        //    if (got_new_gps_) {
        //        auto gps_noise = noiseModel::Isotropic::Sigma(3, 1.0);
        //        GPSFactor gps_factor(X(next_index), Point3(gps_x, gps_y, gps_z), gps_noise);
        //        graph_.add(gps_factor);
        //    }
        // 
        //    Similarly, you could add a depth factor, DVL factor, etc.

        // 5. Update iSAM2
        ISAM2Result result = isam_->update(graph_, initial_estimate_);
        // After calling update, we should clear the “temp” graph_ and initial_estimate_:
        graph_.resize(0);
        initial_estimate_.clear();

        // 6. Get the updated estimate from iSAM2
        Values updated_values = isam_->calculateEstimate();
        Pose3 latest_pose = updated_values.at<Pose3>(X(next_index));
        Vector3 latest_vel = updated_values.at<Vector3>(V(next_index));
        imuBias::ConstantBias latest_bias = updated_values.at<imuBias::ConstantBias>(B(next_index));

        // 7. Store these updated states as “current” going forward
        current_state_ = NavState(latest_pose, latest_vel);
        current_bias_ = latest_bias;

        // 8. Reset the preintegration with the *new* bias
        imu_preintegrated_->resetIntegrationAndSetBias(latest_bias);

        // 9. Advance the “current index”
        current_index_ = next_index;

        // (You might publish this updated pose somewhere.)
        RCLCPP_INFO(this->get_logger(), "Keyframe %d -> Pose: [%f, %f, %f]", 
                    current_index_,
                    latest_pose.translation().x(),
                    latest_pose.translation().y(),
                    latest_pose.translation().z());
    }


    std::shared_ptr<PreintegratedCombinedMeasurements::Params> imu_params(){
        double accel_noise_sigma = 1e-12;//0.0003924;
        double gyro_noise_sigma = 1e-12;//0.000205689024915;
        double accel_bias_rw_sigma = 1e-12;//0.004905;
        double gyro_bias_rw_sigma = 1e-12;//0.000001454441043;
        Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
        Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
        Matrix33 integration_error_cov = I_3x3 * 1e-12;//1e-8;  // error committed in integrating position from velocities
        Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
        Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
        Matrix66 bias_acc_omega_init = I_6x6 * 1e-12;//1e-5;  // error in the bias used for preintegration
        auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
        imu_params->accelerometerCovariance = measured_acc_cov;  // acc white noise in continuous
        imu_params->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
        // should be using 2nd order integration
        // PreintegratedRotation params:
        imu_params->gyroscopeCovariance = measured_omega_cov;  // gyro white noise in continuous
        // PreintegrationCombinedMeasurements params:
        imu_params->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
        imu_params->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
        imu_params->biasAccOmegaInt = bias_acc_omega_init;
        return imu_params;
        
    };

        void initGraphAndState()
    {
        // For demonstration, let's say we start at identity pose, zero velocity, zero bias
        Pose3 prior_pose = Pose3(Rot3::Quaternion(1.0, 0.0, 0.0, 0.0), Point3(0,0,0));
        Vector3 prior_velocity(0,0,0);
        imuBias::ConstantBias prior_bias; 

        auto pose_noise = noiseModel::Isotropic::Sigma(6, 1e-12); //as 1e-2
        auto velocity_noise = noiseModel::Isotropic::Sigma(3, 1e-12);
        auto bias_noise = noiseModel::Isotropic::Sigma(6, 1e-12);

        
        graph_.addPrior<Pose3>(X(0), prior_pose, pose_noise);
        graph_.addPrior<Vector3>(V(0), prior_velocity, velocity_noise);
        graph_.addPrior<imuBias::ConstantBias>(B(0), prior_bias, bias_noise);

        
        initial_estimate_.insert(X(0), prior_pose);
        initial_estimate_.insert(V(0), prior_velocity);
        initial_estimate_.insert(B(0), prior_bias);

        
        current_state_ = NavState(prior_pose, prior_velocity);
        current_bias_ = prior_bias;

        
        isam_->update(graph_, initial_estimate_);
        graph_.resize(0);
        initial_estimate_.clear();
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
