#include "hydrobatic_localization/SamMotionModel.h"

SamMotionModelWrapper::SamMotionModelWrapper(double dt)
    : dt_(dt)
{
  

    {
        py::module motion_model = py::module::import("smarc_modelling.vehicles.SAM_casadi");
        sam_object_ = motion_model.attr("SAM_casadi")(dt_);
        dynamics_func_ = sam_object_.attr("dynamics")();
    }

}

Eigen::VectorXd SamMotionModelWrapper::Dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {

    Eigen::VectorXd x_dot_eigen;
    py::gil_scoped_acquire acquire; 
    py::object x_dot = dynamics_func_(x, u);

    x_dot_eigen = x_dot.cast<Eigen::VectorXd>();
    return x_dot_eigen;
}

Eigen::VectorXd SamMotionModelWrapper::integrateState(const Eigen::VectorXd& x, const Eigen::VectorXd& control,
                                                      const double& integration_time) const
{

    double remaining_time = integration_time;
    Eigen::VectorXd new_state = x;

    while (remaining_time > 0) {
        double dt_step = std::min(dt_, remaining_time);
        if (dt_step <= 0.0) {
            std::cout<< " [Warning] dt_step is less than or equal to 0.0, breaking the loop." << std::endl;
            break;
        }

        // Call the dynamics
        Eigen::VectorXd x_dot = Dynamics(new_state, control);

        // Euler integration
        new_state = new_state + dt_step * x_dot;

        // Normalize the quaternion
        Eigen::Vector4d q = new_state.segment(3, 4);
        q.normalize();
        new_state.segment(3, 4) = q;

        remaining_time -= dt_step;
    }




    return new_state;
}
