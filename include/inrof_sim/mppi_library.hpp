#ifndef MPPI_LIBRARY_HPP
#define MPPI_LIBRARY_HPP

class MppiControl
{
public:
    MppiControl();
    ~MppiControl();

private:
    double estimate_ref();
    double estimate_smooth();
};

#endif