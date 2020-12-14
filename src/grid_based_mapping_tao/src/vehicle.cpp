#include "vehicle.hpp"


//Generate the discretized car cluster relative to ACS, generate the result to the car_grid as argument
void generate_car_cluster(double ego_x,double ego_y,double ego_yaw,double anchor_x,double ancor_y,std::vector<geometry_msgs::Point> &car_grid)
{ 
    double pos_psy=(-ego_yaw+90)/180*M_PI;  //change to rad and "nordweisend Koordinat"
    //This part is to convert the information about car to the grid
    double fl_ego_x=EGO_AXLE2FRONT;
    double rl_ego_x=-EGO_DIM_X+EGO_AXLE2FRONT;
    double rl_ego_y= EGO_DIM_Y/2;
    double rr_ego_y= -EGO_DIM_Y/2;

    std::vector<double> cluster_x_fill=linspace(rl_ego_x,fl_ego_x,30);
    std::vector<double> cluster_y_fill=linspace(rr_ego_y,rl_ego_y,30); 


    std::vector<double> cluster_x_rot;
    std::vector<double> cluster_y_rot;

    for(int i=0;i<cluster_x_fill.size();i++)
    {
        for(int j=0;j<cluster_y_fill.size();j++)
        {
        cluster_x_rot.push_back(cos(pos_psy)*cluster_x_fill[i]-sin(pos_psy)*cluster_y_fill[j]);
            cluster_y_rot.push_back(sin(pos_psy)*cluster_x_fill[i]+cos(pos_psy)*cluster_y_fill[j]);
        }
        
    }

    std::vector<double> cluster_x;
    std::vector<double> cluster_y;
    double offset2anchor_x=ego_x-anchor_x;//Abstand zwischen ACS und VCS berechnen
    double offset2anchor_y=ego_y-ancor_y;
    for(int i=0; i<cluster_x_rot.size();i++)
    {
        cluster_x.push_back(cluster_x_rot[i]+offset2anchor_x);
    }
    for(int i=0; i<cluster_y_rot.size();i++)
    {
        cluster_y.push_back(cluster_y_rot[i]+offset2anchor_y);
    }

    //Convert point clound to grid fields
    std::vector<double> cluster_x_field;
    std::vector<double> cluster_y_field;
    for(int i=0; i<cluster_x.size();i++)
    {
        cluster_x_field.push_back(uint16_t(floor(cluster_x[i]/GRID_SPACING)));
    }
    for(int i=0; i<cluster_y.size();i++)
    {
        cluster_y_field.push_back(uint16_t(floor(cluster_y[i]/GRID_SPACING)));
    }

    geometry_msgs::Point single_point;

    for(int i=0; i<cluster_x_field.size();i++)
    {
        
        single_point.x=cluster_x_field[i];
        single_point.y=cluster_y_field[i];
        car_grid.push_back(single_point);
    }

}

