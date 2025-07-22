#include "mppi_library.hpp"

/*class function begin*/
MppiControl::MppiControl(const int input_dim, const int sampling_number, const int predict_horizon, const std::vector<double> &max_input_value, const double dt)
    : input_dim_(input_dim),
    sampling_number_(sampling_number),
    predict_horizon_(predict_horizon),
    max_input_value_(max_input_value),
    control_cycle_(dt)
{
    
}
MppiControl::~MppiControl()
{

}
/*class function end*/

/*main progress begin*/
std::vector<double> MppiControl::run(std::vector<double> &init_state, Eigen::VectorXd mu, Eigen::MatrixXd sigma)
{
    std::vector<Eigen::MatrixXd> input_array(sampling_number_);
    std::vector<Eigen::MatrixXd> state_array(sampling_number_);
    Eigen::VectorXd S_array(sampling_number_);
    for (size_t i = 0; i < this->sampling_number_; i++) {
        Eigen::VectorXd init_state_eigen = Eigen::Map<Eigen::VectorXd>(&init_state[0], init_state.size());
        input_array[i] = this->generate_input_array(mu, sigma);
        state_array[i] = this->generate_model_state(
            input_array[i],
            init_state_eigen
        );

        S_array(i) = calc_evaluation();
    }
    double S_ref = S_array.minCoeff();

    Eigen::VectorXd weight(sampling_number_);
    Eigen::VectorXd weight_normal(sampling_number_);
    double iota;
    weight = (-(S_array - Eigen::VectorXd::Ones(sampling_number_)*S_ref) / iota).array().exp();
    double weight_sum = weight.sum();
    weight_normal = weight / weight_sum;

    Eigen::VectorXd input(input_dim_);
    input.setZero();
    for (size_t i = 0; i < sampling_number_; i++) input += weight_normal(i) * input_array[i].row(0).transpose();

    std::vector<double> return_input(input.data(), input.data() + input.size());
    return return_input;
}
/*main progress end*/

/*estimate func begin*/
double MppiControl::calc_evaluation()
{

}
double MppiControl::evaluate_ref(const Eigen::VectorXd &value, const Eigen::VectorXd &value_ref)
{

}
double MppiControl::evaluate_smooth(const Eigen::VectorXd &value)
{

}
/*estimate func end*/

/*generate state begin*/
Eigen::MatrixXd MppiControl::generate_model_state(const Eigen::MatrixXd &input_array, const Eigen::VectorXd &init_state)
{
    Eigen::MatrixXd state_array(this->predict_horizon_, this->input_dim_);
    state_array.row(0) = init_state;
    for (size_t i = 1; i < predict_horizon_; i++) state_array.row(i) = this->model(input_array.row(i), state_array.row(i-1));
    return state_array;
}
Eigen::VectorXd MppiControl::model(const Eigen::VectorXd &input, const Eigen::VectorXd &pre_state)
{
    Eigen::VectorXd post_state(this->input_dim_);
    post_state = pre_state + this->control_cycle_*input;
    return post_state;
}
/*generate state end*/

/*util function begin*/
double MppiControl::clamp(const double value, const double max_value, const double min_value)
{
    float clamp_number = std::min(std::max(value, min_value), max_value);
    return clamp_number;
}
/*util function end*/

/*generate input begin*/
Eigen::VectorXd MppiControl::sample_multivariate_normal(
    const Eigen::VectorXd &mean,
    const Eigen::MatrixXd &cov,
    std::mt19937 &gen
)
{
    std::normal_distribution<> dist(0.0, 1.0);
    Eigen::VectorXd z(mean.size());
    for (int i = 0; i < mean.size(); ++i) z(i) = dist(gen);
    Eigen::MatrixXd L = cov.llt().matrixL();
    return mean + L*z;
}

Eigen::MatrixXd MppiControl::generate_input_array(Eigen::VectorXd mu, Eigen::MatrixXd sigma)
{
    Eigen::MatrixXd input_array(this->predict_horizon_, this->input_dim_);

    std::random_device rd;
    std::mt19937 gen(rd());
    for (size_t i = 0; i < this->predict_horizon_; i++)
    {
        input_array.row(i) = this->sample_multivariate_normal(mu, sigma, gen);
        for (size_t j = 0; j < this->input_dim_; j++) input_array(i, j) = this->clamp(input_array(i, j), max_input_value_[j], -max_input_value_[j]);
    }
    return input_array;
}
/*generate input end*/