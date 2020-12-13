

#include "matlab_mimic.hpp"

int8_t sign(int32_t x)
{
    if (x>0) return 1;
    else if (x<0) return -1;
    return 0;
}

std::vector<double> linspace(double a,double b, int n)
{
    std::vector<double> array;
    double step=(b-a)/(n-1);
    while(a<=b)
        {
            array.push_back(a);
            a+=step;
        }
    return array;
}
