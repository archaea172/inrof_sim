#include "mppi_library.hpp"

/*class function begin*/
MppiControl::MppiControl(const int input_dim, const int sampling_number, const int predict_horizon)
    : input_dim_(input_dim), sampling_number_(sampling_number), predict_horizon_(predict_horizon)
{
    
}
MppiControl::~MppiControl()
{

}
/*class function end*/

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
/*generate input end*/