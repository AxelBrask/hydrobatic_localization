#ifndef SAMMOTIONMODEL_H
#define SAMMOTIONMODEL_H

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
//Header for converting Eigen to numpy in function calls
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <gtsam/navigation/NavState.h>
#include <deque>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
namespace py = pybind11;

/**
 * @brief Wrapper class for the SAM motion model
 */
class SamMotionModelWrapper {

public:         
        /** 
        * @brief Constructor for the SAM motion model, and the function objects
        * @param dt: time step for the motion model
        */
        SamMotionModelWrapper(double dt): dt_(dt) {
                py::module motion_model = py::module::import("smarc_modelling.vehicles.SAM_casadi");
                sam_object_ = motion_model.attr("SAM_casadi")(dt_);
                dynamics_func_ = sam_object_.attr("dynamics")();
                prev_control_ = Eigen::VectorXd::Zero(6);
                // jacobian_func_ = sam_object_.attr("linear_dynamics")();
        }
        /**
         * @brief Function to get the dynamics of the SAM motion model
         * @param state: current state of the system [eta, nu, u]
         * @param control: control input to the system
         * @return Eigen::VectorXd: the derivative of the state
         */
        Eigen::VectorXd Dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
                py::object x_dot = dynamics_func_(x, u);
                Eigen::VectorXd x_dot_eigen = x_dot.cast<Eigen::VectorXd>();
                return x_dot_eigen; // return the derivative 

        }

        /**
         * @brief Function to set the previous control input
         * @param control: the previous control input
         */
        void setPrevControl(const Eigen::VectorXd& control) {
                prev_control_ = control;
        }      

        /**
         * @brief Fucntion to get the previous control input
         * @return Eigen::VectorXd: the previous control input
         */
        Eigen::VectorXd getPrevControl() {
                return prev_control_;
        }
        // /**
        //  * @brief Function to get Jacobian with respect to Pose3
        //  * @param state: current state of the system [19x1]
        //  * @param control: control input to the system [6x1]
        //  * @return Eigen::MatrixXd: the Jacobian matrix with respect to the state vector
        //  */
        // Eigen::MatrixXd getJacobianPose3(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
        //         Eigen::MatrixXd jacobian_eigen = jacobian_func_.cast<Eigen::MatrixXd>();
        //         return jacobian_eigen; 
        // }

        /**
         * @brief Integrates the state using the SAM motion model, the time step is set in the constructor of the motion model
         * @param x: current state of the system 
         * @param control: control input to the system
         * @param integration_time: the time to integrate the state
         * @return Eigen::VectorXd: the new state of the system
         */
        Eigen::VectorXd integrateState(const Eigen::VectorXd& x, const Eigen::VectorXd& control, const double& integration_time) {
                double remaining_time = integration_time;
                Eigen::VectorXd new_state = x;
                setPrevControl(control);

                while (remaining_time > 0) {
                        // Use the smaller of dt_ or the remaining time for the current step.
                        double dt_step = std::min(dt_, remaining_time);
                        Eigen::VectorXd x_dot = Dynamics(new_state, control);
                        new_state = new_state + dt_step * x_dot;
                        // Normalize the quaternion 
                        Eigen::Vector4d q = new_state.segment(3, 4);
                        q.normalize();
                        new_state.segment(3, 4) = q;
                        remaining_time -= dt_step;
                }
        return new_state;
        }

        /**     
         * @brief Function to convert the gtsam::NavState and gyro measurements to eta and nu for the motion model
         * @param state: the current NavState state from the optimizer in the navigation frame
         * @param gyro: the raw gyro measurements from the IMU
         * @return Eigen::VectorXd: the state vector, eta and nu. The linear and angluar velocities will tbe transformed to the base link frame
         */
        Eigen::VectorXd stateToVector(const gtsam::NavState& state, const gtsam::Vector3 gyro) {

                // Translation and rotation this needs to be converted from ENU to NED
                Eigen::Matrix3d R_e2n;       
                R_e2n << 0, 1, 0,
                        1, 0, 0,
                        0, 0, -1;
                
                // Convert the translation: from ENU to Ned
                Eigen::VectorXd t_enu = state.pose().translation();
                Eigen::VectorXd t_ned;
                t_ned << t_enu.y(), t_enu.x(), -t_enu.z();
                
                // Get the ENU rotation matrix from the nav state
                Eigen::Matrix3d R_enu = state.pose().rotation().matrix();
                // Convert it to NED by applying the transformation on both sides
                Eigen::Matrix3d R_ned = R_e2n * R_enu * R_e2n.transpose();
                // Convert the rotation matrix to a quaternion (in NED)
                Eigen::Quaterniond q_ned(R_ned);
                
                // Fill the state vector: first three elements are translation, 
                // next four are the quaternion (w, x, y, z)
                Eigen::VectorXd eta(7);
                eta << t_ned(0), t_ned(1), t_ned(2),
                q_ned.w(), q_ned.x(), q_ned.y(), q_ned.z();
                // Linear and angular velocity
                Eigen::VectorXd nu(6);
                //Velcoity in the base link frame
                Eigen::VectorXd base_link_velocity = state.pose().rotation().transpose().matrix()*state.velocity();
                nu << base_link_velocity.x(), base_link_velocity.y(), base_link_velocity.z(), gyro.x(), gyro.y(), gyro.z();
                Eigen::VectorXd state_vector(eta.size() + nu.size());
                state_vector << eta, nu;
                return state_vector;
        }

        /**
         * @brief Function to add control inputs to the control seqeuence
         * @param u: the control input is either [4x1] or [2x1] depending if fb topic or command topic
         * @param timestamp: the timestamp of the message, this will be used to calculate the integration time
         * @param isThrusterVector: if true, u needs to be [2x1] and is a thruster vector cmd message, if false [4x1] and a fb control
         */
        void controlToQueue(const Eigen::VectorXd& u, const double& timestamp, const bool& isThrusterVector, rclcpp::Logger logger){
                // add the first input to the qeue
                if(control_queue_.empty()){
                        controlSequence new_control;
                        new_control.u = Eigen::VectorXd::Zero(6); // Initialize with zeros
                        if (isThrusterVector) {
                        new_control.u.segment<2>(0) = u;
                        } else {
                        new_control.u.segment<2>(0) = u.segment<2>(0);
                        new_control.u.segment<2>(4) = u.segment<2>(2);
                        }
                        new_control.timestamp = timestamp;
                        std::stringstream ss;
                        ss << "New control added: " << new_control.u.transpose() << ", timestamp: " << new_control.timestamp;
                        RCLCPP_INFO(logger, "%s", ss.str().c_str());
                        control_queue_.push_back(new_control);
                        return;
                }
                //Get the latest control in the queue
                controlSequence latest_control = control_queue_.back();

                // Update the control input, if the latest control is a thruster vector, update only element 2 and 3
                if(isThrusterVector){
                        latest_control.u.segment<2>(2) = u;

                }
                // If the latest control is a feedback control, update the first 2 elements and the last 2 elements
                else{
                        latest_control.u.segment<2>(0) = u.segment<2>(0);
                        latest_control.u.segment<2>(4) = u.segment<2>(2);
                }
                latest_control.timestamp = timestamp;
                // std::stringstream ss;
                // ss << "Updated latest control: " << latest_control.u.transpose() << ", timestamp: " << latest_control.timestamp;
                // RCLCPP_INFO(logger, "%s", ss.str().c_str());
                // RCLCPP_INFO(logger, "Control queue size: %d", control_queue_.size());
                control_queue_.push_back(latest_control);

        }
        /**
         * @brief Function to pre integrate the state using the SAM motion model and the control sequence
         * @param x: the current state vector
         * @param start_time: the start time of the integration
         * @param end_time: the end time of the integration
         * @return Eigen::VectorXd: the pre integrated state
         */
        Eigen::VectorXd preIntegrateState(const Eigen::VectorXd& x, const double start_time, const double end_time, rclcpp::Logger logger) {
                if(control_queue_.empty()) {
                        return x;
                }
                Eigen::VectorXd integratedState = x;
                double currentTime = start_time;
                // Integrate the state until the first control input, by using the previous control input if the timestamp does not mathc the start time
                if(control_queue_[1].timestamp > currentTime) {
                        double dt = control_queue_[1].timestamp - currentTime;
                        // RCLCPP_INFO(logger,"dt in initial gap %f",dt);
                        // RCLCPP_INFO(logger,"timestamp  and current time %f %f",control_queue_.front().timestamp,currentTime);
                        // control_queue_.front().u will be the previous control input since we never pop the queue when integrating from the last control to end_time
                        integratedState = integrateState(integratedState, control_queue_.front().u,dt);
                        // std::stringstream ss;
                        // ss << "Control used: " << control_queue_.front().u.transpose() << ", timestamp: " << control_queue_.front().timestamp;
                        // RCLCPP_INFO(logger, "%s", ss.str().c_str());
                        currentTime = control_queue_[1].timestamp;
                        control_queue_.pop_front();
                }
                //integrate the state until the integration time, but keep the last control input
                while(control_queue_.size() >= 2 && control_queue_[1].timestamp <= end_time ) {

                        // get time difference to the next control input
                        double dt = control_queue_[1].timestamp - currentTime;
                        // RCLCPP_INFO(logger,"dt %f",dt);
                        integratedState = integrateState(integratedState, control_queue_.front().u, dt);
                        // std::stringstream ss;   
                        // ss << "Control used: " << control_queue_.front().u.transpose() << ", timestamp: " << control_queue_.front().timestamp;
                        // RCLCPP_INFO(logger, "%s", ss.str().c_str());
                        currentTime = control_queue_[1].timestamp;
                        control_queue_.pop_front();

                }
                // Now we have to integrate the last dynamics from the last control input to the integration time by using the last control input
                double dt = end_time - currentTime;
                RCLCPP_INFO(logger,"dt in final gap %f",dt);
                if(dt > 0) {
                        integratedState = integrateState(integratedState, control_queue_.front().u, dt);
                        //log the contorl input used
                        // std::stringstream ss;
                        // ss << "Control used: " << control_queue_.front().u.transpose() << ", timestamp: " << control_queue_.front().timestamp;
                        // RCLCPP_INFO(logger, "%s", ss.str().c_str());
                }
                std::stringstream ss;
                ss << "Updated state: " << integratedState.transpose();
                RCLCPP_INFO(logger, "%s", ss.str().c_str());
                return integratedState;

        }

        
  

private:
        // struct for the control queue for Sam Motion model
        struct controlSequence {
                double timestamp;
                Eigen::VectorXd u;
        };
        double dt_;
        py::object sam_object_;
        Eigen::VectorXd prev_control_;
        py::object dynamics_func_;
        py::object jacobian_func_;
        std::deque<controlSequence> control_queue_;

};



#endif // SAMMOTIONMODEL_H