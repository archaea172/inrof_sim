#ifndef MPPI_LIBRARY_HPP
#define MPPI_LIBRARY_HPP

#include <Eigen/Dense>

class MppiControl
{
public:
    MppiControl(const int input_dim);
    ~MppiControl();

private:
    int input_dim_;

    double estimate_ref();
    double estimate_smooth();
    Eigen::VectorXd model(Eigen::MatrixXd input_array, Eigen::VectorXd init_state);
    double clamp(const double value, const double max_value, const double min_value);
};

#endif