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
        const std::vector<double> &max_input_value,
        const double dt,
        std::vector<double> gain_list
    );
    ~MppiControl();

    std::vector<double> run(std::vector<double> &init_state, std::vector<double> &goal_state, Eigen::VectorXd &mu, Eigen::MatrixXd &sigma, double iota);

private:
    /*declare value begin*/
    const int input_dim_;
    const int sampling_number_;
    const int predict_horizon_;
    std::vector<double> max_input_value_;
    const double control_cycle_;
    Eigen::VectorXd gain_vector;
    std::mt19937 gen_;
    /*declare value end*/

    /*estimate func begin*/
    double calc_evaluation(Eigen::MatrixXd InputList, Eigen::MatrixXd StateList, Eigen::MatrixXd GoalPose);
    double evaluate_ref(const Eigen::VectorXd &value, const Eigen::VectorXd &value_ref);
    double evaluate_smooth(const Eigen::VectorXd &value);
    /*estimate func end*/

    /*generate state begin*/
    Eigen::MatrixXd generate_model_state(const Eigen::MatrixXd &input_array, const Eigen::VectorXd &init_state);
    Eigen::VectorXd model(const Eigen::VectorXd &input, const Eigen::VectorXd &pre_state);
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
        Eigen::VectorXd mu,
        Eigen::MatrixXd sigma
    );
    /*generate input end*/
};

#endif