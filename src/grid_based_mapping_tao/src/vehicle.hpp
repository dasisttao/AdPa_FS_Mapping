#ifndef VEHICLE_HPP_INCLUDE
#define VEHICLE_HPP_INCLUDE
#include "grid_based_mapping_tao/parameter_grid.h"
#include "matlab_mimic/matlab_mimic.hpp"
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
  double get_rate_pitch() const {return rate_pitch;}
  double get_pitch() const {return pitch;}

  void set_x(double x){egopos_x_=x;}
  void set_y(double y){egopos_y_=y;}
  void set_psi(double psi){egopos_psi_=psi;}
  void set_rate_pitch(double w){rate_pitch=w;}
  void set_pitch(double p){pitch=p;}

private:
  double egopos_x_;
  double egopos_y_;
  double egopos_psi_;  // unit [degree]  frame UTM 
  double rate_pitch;
  double pitch;
};


void generate_car_cluster(double ego_x,double ego_y,double ego_yaw,double anchor_x,double ancor_y,std::vector<geometry_msgs::Point> &car_grid);



#endif