#include "grid_based_mapping_tao/parameter_grid.h"

#ifndef OCCUPANCY_GRID_2D_HPP_INCLUDE
#define OCCUPANCY_GRID_2D_HPP_INCLUDE



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


#endif