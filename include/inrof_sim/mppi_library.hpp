#ifndef MPPI_LIBRARY_HPP
#define MPPI_LIBRARY_HPP

#include <Eigen/Dense>
#include <random>

class MppiControl
{
public:
    MppiControl(
        const int input_dim, 
        const int sampling_number, 
        const int predict_horizon,
        const std::vector<double> &max_input_value
    );
    ~MppiControl();

    std::vector<double> run(std::vector<double> init_state);

private:
    /*declare value begin*/
    const int input_dim_;
    const int sampling_number_;
    const int predict_horizon_;
    std::vector<double> max_input_value_;
    /*declare value end*/

    /*estimate func begin*/
    double estimate_ref();
    double estimate_smooth();
    /*estimate func end*/

    /*generate state begin*/
    Eigen::MatrixXd generate_model_state(Eigen::MatrixXd &input_array, Eigen::VectorXd &init_state);
    Eigen::VectorXd model(Eigen::VectorXd &input, Eigen::VectorXd &pre_state);
    /*generate state end*/

    /*util function begin*/
    double clamp(const double value, const double max_value, const double min_value);
    /*util function end*/

    /*generate input begin*/
    Eigen::VectorXd sample_multivariate_normal(
        const Eigen::VectorXd &mean,
        const Eigen::MatrixXd &cov,
        std::mt19937 &gen
    );

    Eigen::MatrixXd generate_input_array(
        
    );
    /*generate input end*/
};

#endif