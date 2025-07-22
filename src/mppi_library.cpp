#include "mppi_library.hpp"
#include <math.h>

// omni_simulate
void omni_calc(float theta,float vx,float vy,float omega,float *w0,float *w1,float *w2,float *w3){
    const float a0 = M_PI/180*45;
    const float a1 = M_PI/180*135;
    const float a2 = M_PI/180*225;
    const float a3 = M_PI/180*315;
    const float r = 0.03;//m
    const float R = 0.144;//m
    float v[3] = {vx, vy, omega};
    float sint = sin(theta);
    float cost = cos(theta);

    float arr[4][3] =
    {{-cos(a0)*sint-sin(a0)*cost, cos(a0)*cost-sin(a0)*sint, R},
    {-cos(a1)*sint-sin(a1)*cost, cos(a1)*cost-sin(a1)*sint, R},
    {-cos(a2)*sint-sin(a2)*cost, cos(a2)*cost-sin(a2)*sint, R},
    {-cos(a3)*sint-sin(a3)*cost, cos(a3)*cost-sin(a3)*sint, R}};

    *w0 = (arr[0][0] * v[0] + arr[0][1] * v[1] + arr[0][2] * v[2]) / r;
    *w1 = (arr[1][0] * v[0] + arr[1][1] * v[1] + arr[1][2] * v[2]) / r;
    *w2 = (arr[2][0] * v[0] + arr[2][1] * v[1] + arr[2][2] * v[2]) / r;
    *w3 = (arr[3][0] * v[0] + arr[3][1] * v[1] + arr[3][2] * v[2]) / r;
}

/*class function begin*/
MppiControl::MppiControl(const int input_dim, const int sampling_number, const int predict_horizon, const std::vector<double> &max_input_value, const double dt, std::vector<double> gain_list)
    : input_dim_(input_dim),
    sampling_number_(sampling_number),
    predict_horizon_(predict_horizon),
    max_input_value_(max_input_value),
    control_cycle_(dt)
{
    gain_vector = Eigen::Map<Eigen::VectorXd>(&gain_list[0], gain_list.size());
}
MppiControl::~MppiControl()
{

}
/*class function end*/

/*main progress begin*/
std::vector<double> MppiControl::run(std::vector<double> &init_state, Eigen::VectorXd &mu, Eigen::MatrixXd &sigma)
{
    std::vector<Eigen::MatrixXd> input_array(sampling_number_);
    std::vector<Eigen::MatrixXd> state_array(sampling_number_);
    Eigen::VectorXd S_array(sampling_number_);
    Eigen::VectorXd init_state_eigen = Eigen::Map<Eigen::VectorXd>(&init_state[0], init_state.size());
    for (size_t i = 0; i < this->sampling_number_; i++) {
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


    double S = 0;
    return S;
}
double MppiControl::evaluate_ref(const Eigen::VectorXd &value, const Eigen::VectorXd &value_ref)
{
    double sum_diff = 0;
    sum_diff += (value - value).array().cwiseAbs2().sum();
    return sum_diff;
}
double MppiControl::evaluate_smooth(const Eigen::VectorXd &value)
{
    double sum_diff = 0;
    for (size_t i = 1; i < (size_t)value.size(); i++) sum_diff += std::pow(value(i) - value(i-1), 2);
    return sum_diff;
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