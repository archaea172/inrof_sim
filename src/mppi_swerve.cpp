#include "mppi_swerve.hpp"

Eigen::VectorXd MppiSwerve::model(const Eigen::VectorXd &input, const Eigen::VectorXd &pre_state)
{
    Eigen::VectorXd post_state(this->input_dim_);
    post_state = pre_state + this->control_cycle_*input;
    return post_state;
}

Eigen::MatrixXd MppiSwerve::generate_model_state(const Eigen::MatrixXd &input_array, const Eigen::VectorXd &init_state)
{
    Eigen::MatrixXd state_array(this->predict_horizon_, this->input_dim_);
    state_array.row(0) = init_state;
    for (size_t i = 1; i < (size_t)predict_horizon_; i++) state_array.row(i) = this->model(input_array.row(i), state_array.row(i-1)).transpose();
    return state_array;
}