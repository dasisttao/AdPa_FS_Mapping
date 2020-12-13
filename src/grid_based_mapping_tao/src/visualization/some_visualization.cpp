#include "some_visualization.hpp"



//visualize Gridcells
void visualize_girdcells(const std::vector<geometry_msgs::Point>& cells,const std::string& frame_id,float grid_cell_width,float grid_cell_height,
                        ros::Publisher pub,double offset_x,double offset_y)
{ 
    nav_msgs::GridCells vis_grid;
    vis_grid.header.stamp=ros::Time::now();
    vis_grid.header.frame_id=frame_id;
    vis_grid.cell_width=grid_cell_width;
    vis_grid.cell_height=grid_cell_height;


    for(int i=0;i<cells.size();i++)
    {
        vis_grid.cells.push_back(cells[i]);
    }

    if(offset_x!=0)
    {
        for(int i=0;i<vis_grid.cells.size();i++)
        {
            vis_grid.cells[i].x-=offset_x;
        }
    }
    if(offset_y!=0)
    {
        for(int i=0;i<vis_grid.cells.size();i++)
        {
            vis_grid.cells[i].y-=offset_y;
        }
    }

  pub.publish(vis_grid);
}