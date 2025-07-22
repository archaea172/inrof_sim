#include "mppi_library.hpp"

/*class function begin*/
MppiControl::MppiControl(const int input_dim)
    : input_dim_(input_dim)
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