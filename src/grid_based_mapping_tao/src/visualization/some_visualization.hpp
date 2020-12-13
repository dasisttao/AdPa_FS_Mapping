#ifndef SOME_VISUALIZATION_HPP_INCLUDE
#define SOME_VISUALIZATION_HPP_INCLUDE


#include "grid_based_mapping_tao/parameter_grid.h"
void visualize_girdcells(const std::vector<geometry_msgs::Point>& cells,const std::string& frame_id,float grid_cell_width,float grid_cell_height,
                        ros::Publisher pub,double offset_x,double offset_y);
#endif