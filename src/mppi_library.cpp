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