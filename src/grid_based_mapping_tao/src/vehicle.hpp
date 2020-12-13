#ifndef VEHICLE_HPP_INCLUDE
#define VEHICLE_HPP_INCLUDE
#include "grid_based_mapping_tao/parameter_grid.h"

class Vehicle{
public:
  Vehicle()
  {
    egopos_x_=0.0;
    egopos_y_=0.0;
    egopos_psi_=0.0;
  }
  Vehicle(double x,double y,double psi)
  {
    egopos_x_=x;
    egopos_y_=y;
    egopos_psi_=psi;
  }
  //~Vehicle();

  double get_x() const {return egopos_x_;}
  double get_y() const {return egopos_y_;}
  double get_psi() const {return egopos_psi_;}

  void set_x(double x){egopos_x_=x;}
  void set_y(double y){egopos_y_=y;}
  void set_psi(double psi){egopos_psi_=psi;}

private:
  double egopos_x_;
  double egopos_y_;
  double egopos_psi_;  // unit [degree]  frame UTM 

};





#endif