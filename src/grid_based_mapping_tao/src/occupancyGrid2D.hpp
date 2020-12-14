

#ifndef OCCUPANCY_GRID_2D_HPP_INCLUDE
#define OCCUPANCY_GRID_2D_HPP_INCLUDE

#include "grid_based_mapping_tao/parameter_grid.h"
#include "matlab_mimic/matlab_mimic.hpp"

class oG2D //occupancyGrid2D
{
  public:
    oG2D(double resolution,int32_t width,int32_t height,geometry_msgs::Pose origin){
      oG.info.resolution=resolution;
      oG.info.width=width;
      oG.info.height=height;
      oG.info.origin=origin;
      //oG.info.map_load_time=ros::Time::now();
    };
    ~oG2D(){};
    void visualization(const std::vector<int8_t>& probability_vector,const ros::Publisher& pub,const std::string& frame_id);

    void setOrigin(geometry_msgs::Pose origin)
    {oG.info.origin=origin;}

  private:
  nav_msgs::OccupancyGrid oG;
  
};




class ABC
{
  public:
    ABC(){
        current_anchor_x=0.0;
        current_anchor_y=0.0;

        history_anchor_x=0.0;
        history_anchor_y=0.0;

        grid_border_x1_cell=(GRID_SIZE_X*150/400)-1;
        grid_border_x2_cell=(GRID_SIZE_X*250/400)-1;
        grid_border_y1_cell=(GRID_SIZE_Y*150/400)-1;
        grid_border_y2_cell=(GRID_SIZE_Y*250/400)-1;

        grid_border_x1_m=grid_border_x1_cell*GRID_SPACING;
        grid_border_x2_m=grid_border_x2_cell*GRID_SPACING;
        grid_border_y1_m=grid_border_y1_cell*GRID_SPACING;
        grid_border_y2_m=grid_border_y2_cell*GRID_SPACING;

    };
    //~ABC(){};
    double get_current_anchor_x() const {return current_anchor_x;}
    double get_current_anchor_y() const {return current_anchor_y;}

    void set_current_anchor_x(double x){current_anchor_x=x;}
    void set_current_anchor_y(double y){current_anchor_y=y;}

    double get_history_anchor_x() const {return history_anchor_x;}
    double get_history_anchor_y() const {return history_anchor_y;}

    void set_history_anchor_x(double x){history_anchor_x=x;}
    void set_history_anchor_y(double y){history_anchor_y=y;}

    int16_t get_borderoFcar_x1_cell() const {return grid_border_x1_cell;}
    int16_t get_borderoFcar_x2_cell() const {return grid_border_x2_cell;}
    int16_t get_borderoFcar_y1_cell() const {return grid_border_y1_cell;}
    int16_t get_borderoFcar_y2_cell() const {return grid_border_y2_cell;}

    double get_borderoFcar_x1_m() const {return grid_border_x1_m;}
    double get_borderoFcar_x2_m() const {return grid_border_x2_m;}
    double get_borderoFcar_y1_m() const {return grid_border_y1_m;}
    double get_borderoFcar_y2_m() const {return grid_border_y2_m;}
    

    //when car touch the border,grid should move; The distance it move are:
    //unit m

    int16_t get_grid_trans_x_cell() const {return grid_border_x2_cell-grid_border_x1_cell;}
    int16_t get_grid_trans_y_cell() const {return grid_border_y2_cell-grid_border_y1_cell;}

    double get_grid_trans_x_m() const {return grid_border_x2_m-grid_border_x1_m;}
    double get_grid_trans_y_m() const {return grid_border_y2_m-grid_border_y1_m;}

    

  private:
    //This is the origin of anchor coordinate system with respect to world coordinate frame
    //unit m
    double current_anchor_x;
    double current_anchor_y;

    double history_anchor_x;
    double history_anchor_y;

    //This is border of car movement enviroment to make sure the car is alwasy on the center of the Gird
    //unit m
    double grid_border_x1_m;
    double grid_border_x2_m;
    double grid_border_y1_m;
    double grid_border_y2_m;
  
    int16_t grid_border_x1_cell;
    int16_t grid_border_x2_cell;
    int16_t grid_border_y1_cell;
    int16_t grid_border_y2_cell;
    


};

void binary_bayes(float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y],float max_l,float min_l,float unknow_l);//max_l and min_l is e.g 99=log(99.0/(100.0-99.0)).






#endif

