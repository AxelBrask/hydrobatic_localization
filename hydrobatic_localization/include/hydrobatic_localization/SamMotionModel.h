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
                setPrevControl(u);
                return x_dot_eigen; // return the derivative 

        }

        /**
         * @brief Function to set the previous control input
         * @param control: the previous control input
         */
        void setPrevControl(const Eigen::VectorXd& control) {
                prev_control_ = control;
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
         * @return Eigen::VectorXd: the new state of the system
         */
        Eigen::VectorXd integrateState(const Eigen::VectorXd& x, const Eigen::VectorXd& control, const double& dt) {
                
                Eigen::VectorXd x_dot = Dynamics(x, control);
                Eigen::VectorXd new_state = x + dt * x_dot;
                Eigen::Vector4d q = new_state.segment(3, 4);
                q.normalize();
                new_state.segment(3, 4) = q;

                return new_state;

                }

        /**     
         * @brief Function to convert the gtsam::NavState and gyro measurements to eta and nu for the motion model
         * @param state: the current NavState state from the optimizer in the navigation frame
         * @param gyro: the raw gyro measurements from the IMU
         * @return Eigen::VectorXd: the state vector, eta and nu. The linear and angluar velocities will tbe transformed to the base link frame
         */
        Eigen::VectorXd stateToVector(const gtsam::NavState& state, const gtsam::Vector3 gyro) {

                // Translation and rotation
                Eigen::VectorXd eta(7);
                Quaternion quat = state.pose().rotation().toQuaternion();
                eta << state.pose().translation().x(), state.pose().translation().y(), state.pose().translation().z(), quat.x(), quat.y(), quat.z(), quat.w();
                // Linear and angular velocity
                Eigen::VectorXd nu(6);
                //Velcoity in the base link frame
                Vector3 base_link_velocity = state.pose().rotation().transpose().matrix()*state.velocity();
                nu << base_link_velocity.x(), base_link_velocity.y(), base_link_velocity.z(), gyro.x(), gyro.y(), gyro.z();
                Eigen::VectorXd state_vector(eta.size() + nu.size());
                state_vector << eta, nu;
                return state_vector;
        }

  

private:
        double dt_;
        py::object sam_object_;
        Eigen::VectorXd prev_control_;
        py::object dynamics_func_;
        py::object jacobian_func_;
};



#endif // SAMMOTIONMODEL_H