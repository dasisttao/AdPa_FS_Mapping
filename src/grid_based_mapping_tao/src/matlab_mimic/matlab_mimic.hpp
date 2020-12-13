#ifndef MATLAB_MIMIC_HPP_INCLUDE
#define MATLAB_MIMIC_HPP_INCLUDE

#include "grid_based_mapping_tao/parameter_grid.h"


int8_t sign(int32_t x); //if argument>0, return 1; argument<0, return -1; argument==0, return 0

std::vector<double> linspace(double a,double b, int n);//first and second argument are both range point, the third argument is the number of inserting point;

#endif 
