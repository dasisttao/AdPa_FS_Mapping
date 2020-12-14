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


void binary_bayes(float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y],float max_l,float min_l,float unknow_l)//max_l and min_l is e.g 99=log(99.0/(100.0-99.0));
{
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int k=0;k<GRID_SIZE_Y;k++)
            {
                if (((*current_grid)[i][k])!=unknow_l)
                {
                    (*history_grid)[i][k]=(*current_grid)[i][k]+(*history_grid)[i][k];
                }
                else 
                {
                    (*history_grid)[i][k]=(*history_grid)[i][k];
                }
                
                if((*history_grid)[i][k]>max_l)
                {
                    (*history_grid)[i][k]=max_l;
                }
                else if ((*history_grid)[i][k]<min_l)
                {
                    (*history_grid)[i][k]=min_l;
                }
            }
            
}





