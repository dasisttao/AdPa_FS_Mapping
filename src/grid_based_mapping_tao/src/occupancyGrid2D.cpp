
#include "occupancyGrid2D.hpp"

void oG2D::visualization(const std::vector<int8_t>& probability_vector,const ros::Publisher& pub,const std::string& frame_id)
{
    oG.header.frame_id=frame_id;
    oG.header.stamp=ros::Time::now();
    for(auto i=0;i<probability_vector.size();i++)
    {
        oG.data.push_back(probability_vector[i]);
    }
    pub.publish(oG);
}