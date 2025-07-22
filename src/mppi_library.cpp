#include "mppi_library.hpp"

/*class function begin*/
MppiControl::MppiControl(const int input_dim, const int sampling_number, const int predict_horizon, const std::vector<double> &max_input_value)
    : input_dim_(input_dim),
    sampling_number_(sampling_number),
    predict_horizon_(predict_horizon),
    max_input_value_(max_input_value)
{
    
}
MppiControl::~MppiControl()
{

}
/*class function end*/

/*main progress begin*/
std::vector<double> MppiControl::run(std::vector<double> init_state)
{
    std::vector<Eigen::MatrixXd> input_array(sampling_number_);
    std::vector<Eigen::MatrixXd> state_array(sampling_number_);
    
    for (size_t i = 0; i < this->sampling_number_; i++) {
        input_array[i] = this->generate_input_array();
    }

    
}
/*main progress end*/

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

Eigen::MatrixXd MppiControl::generate_input_array()
{
    Eigen::MatrixXd input_array(this->predict_horizon_, this->input_dim_);

    Eigen::VectorXd mu(this->input_dim_);
    mu << 1.0, 1.0, 0.5;

    Eigen::MatrixXd sigma(this->input_dim_, this->input_dim_);
    sigma << 
    1.0, 0.5, 0.2,
    0.5, 1.0, 0.3,
    0.2, 0.3, 1.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    for (size_t i = 0; i < this->predict_horizon_; i++) input_array.row(i) = this->sample_multivariate_normal(mu, sigma, gen);
    
    return input_array;
}
/*generate input end*/