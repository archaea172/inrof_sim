#include "mppi_swerve.hpp"

Eigen::VectorXd MppiSwerve::model(const Eigen::VectorXd &input, const Eigen::VectorXd &pre_state) override
{
    Eigen::VectorXd post_state(this->input_dim_);
    post_state = pre_state + this->control_cycle_*input;
    return post_state;
}

