#ifndef SAMMOTIONMODEL_H
#define SAMMOTIONMODEL_H

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <Eigen/Dense>
#include <deque>
#include <chrono>
#include <iostream>
#include <atomic>    // For std::atomic<int>
#include <thread>    // For std::this_thread::get_id()
#include <algorithm> // For std::min
#include <mutex>    // For std::mutex

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
    SamMotionModelWrapper(double dt);

    /**
     * @brief Function to get the dynamics of the SAM motion model
     * @param x: current state of the system [eta, nu, u]
     * @param control: control input to the system
     * @return Eigen::VectorXd: the derivative of the state
     */
    Eigen::VectorXd Dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;

    /**
     * @brief Integrates the state using the SAM motion model, the time step is set in the constructor
     * @param x: current state of the system 
     * @param control: control input to the system
     * @param integration_time: the time to integrate the state
     * @return Eigen::VectorXd: the new state of the system
     */
    Eigen::VectorXd integrateState(const Eigen::VectorXd& x, const Eigen::VectorXd& control, 
                                   const double& integration_time) const;

private:
    // A helper struct for the control queue (currently unused)
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
    static std::mutex pyMutex;
};

#endif // SAMMOTIONMODEL_H
