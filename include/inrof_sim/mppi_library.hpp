#ifndef MPPI_LIBRARY_HPP
#define MPPI_LIBRARY_HPP

#include <Eigen/Dense>

class MppiControl
{
public:
    MppiControl(const int input_dim, const int sampling_number, const int predict_horizon);
    ~MppiControl();

private:
    /*declare value begin*/
    const int input_dim_;
    const int sampling_number_;
    const int predict_horizon_;
    /*declare value end*/

    /*estimate func begin*/
    double estimate_ref();
    double estimate_smooth();
    /*estimate func end*/
    Eigen::VectorXd model(Eigen::MatrixXd input_array, Eigen::VectorXd init_state);

    double clamp(const double value, const double max_value, const double min_value);
};

#endif