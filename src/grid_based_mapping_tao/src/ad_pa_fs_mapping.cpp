#include "grid_based_mapping_tao/parameter_grid.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include "occupancyGrid2D.hpp"
#include "vehicle.hpp"
#include "matlab_mimic/matlab_mimic.hpp"
#include "visualization/some_visualization.hpp"
/*++++++++++++++++++++++++++This node is doing the following  jobs+++++++++++++++++++++++++++++++++++++++++
    
        Get Message from Topic /VehiclePoseFusionUTM so that we can get the car pose 
        Get Message from Topic /as_tx/point_cloud so that we can get the pcl message

        Let pcl discretized in grid and use bayes filter to estimate the probabilty of the occupization


      Publish the Message to Topic /data2visual so that we can visualize it with rviz.The Message conatains
      car cluster, anchor position and occupancygrid(history_grid)(by a varibale of type std_msgs/UInt8MultiArray)

 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/




bool flag_msg_valid {false};


 //++++++++++++++++++++++++++++++++++++++++++++++++CONFIG/PARAMETER++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint64_t allowed_diff_ts_ns=25000;//0.025s -the allowed difference of timestamp between msg of car_pose and msg of pcl2


namespace CAR_LASER
{
    //double DMAX=10; //max distannce of reachement
    //double DMIN=1;
    std::vector<double> laser_x;
    std::vector<double> laser_y;
    std::vector<double> laser_yaw;

    std::vector<double> angle_resolution;

    std::vector<uint32_t> radiation_num;

    std::vector<int> radiation_num_divider;
    

    std::vector<double> range_min;
    std::vector<double> range_max;
}

using namespace CAR_LASER;


 //++++++++++++++++++++++++++++++++++++++++++++++++global varibale+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Vehicle car;//create global object car 


bool if_get_new_data {false};

//++++++++++++++++++++++++++++Paramenter for Anchor Corrdinate System unit[m] relateive to utm origin+++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++Functions++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void update_grid_when_anchor_moved(float (*grid)[GRID_SIZE_X][GRID_SIZE_Y],float new_are_fullfill,const ABC& acs);

void callback(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg,const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& pcl_msg,uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],ABC* ptracs);
void adjust_ACS(ABC &acs);// according to the current pose of car,update the ACS anchor
void anchor_init(const Vehicle& car,ABC& acs);// according to the initial pose of car, initiliaze the ACS anchor. 



void create_freespace(uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],const std::vector<float>& l,const ABC& acs);
std::vector<float> l;
enum probability {max=0,min=1,unknown=2,fill=3,clear=4};

enum discrete_pcl {unset=0,set=1};

//++++++++++++++++++++++++++++++++++++++++++++++++++MAIN++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
    ros::init(argc,argv,"ad_pa_fs_mapping");
    ros::NodeHandle nh;
    ros::Rate loop_rate(25);

    ABC acs;
    ABC* ptracs;
    ptracs=&acs;

    float param;
    nh.getParam("p_max",param);
    l.push_back(log(param/(100.0-param)));

    nh.getParam("p_min",param);
    l.push_back(log(param/(100.0-param)));

    nh.getParam("p_unknown",param);
    l.push_back(log(param/(100.0-param)));

    nh.getParam("p_fill",param);
    l.push_back(log(param/(100.0-param)));

    nh.getParam("p_clear",param);
    l.push_back(log(param/(100.0-param)));

    
    
    ROS_INFO("l_max: %f",l[probability::max]);
    ROS_INFO("l_min: %f",l[probability::min]);
    ROS_INFO("l_unknown: %f",l[probability::unknown]);
    
    ROS_INFO("l_fill: %f",l[probability::fill]);
    ROS_INFO("l_clear: %f",l[probability::clear]);

    
    if(!nh.getParam("/range_min",range_min))
        {ROS_ERROR("Failed to get range_min from server.");}
    
    for(int i=0;i<range_min.size();i++)
    {
        std::cout<<"range_min["<<i<<"] = "<<range_min[i]<<std::endl;
    }

    if(!nh.getParam("/range_max",range_max))
        {ROS_ERROR("Failed to get range_max from server.");}
    
    for(int i=0;i<range_max.size();i++)
    {
        std::cout<<"range_max["<<i<<"] = "<<range_max[i]<<std::endl;
    }
    if(!nh.getParam("/laser_x",laser_x))
        {ROS_ERROR("Failed to get laser_x from server.");}
    
    for(int i=0;i<laser_x.size();i++)
    {
        std::cout<<"laser_x["<<i<<"] = "<<laser_x[i]<<std::endl;
    }
    if(!nh.getParam("/laser_y",laser_y))
        {ROS_ERROR("Failed to get laser_y from server.");}
    
    for(int i=0;i<laser_y.size();i++)
    {
        std::cout<<"laser_y["<<i<<"] = "<<laser_y[i]<<std::endl;
    }

    if(!nh.getParam("/laser_yaw",laser_yaw))
        {ROS_ERROR("Failed to get laser_yaw from server.");}
    
    for(int i=0;i<laser_yaw.size();i++)
    {
        std::cout<<"laser_yaw["<<i<<"] = "<<laser_yaw[i]<<"*M_PI/180";
        laser_yaw[i]=laser_yaw[i]*M_PI/180;
        std::cout<<"="<<laser_yaw[i]<<std::endl;
    }

    if(!nh.getParam("/radiation_num_divider",radiation_num_divider))
        {ROS_ERROR("Failed to get radiation_num_divider from server.");}
    for(int i=0;i<radiation_num_divider.size();i++)
    {
        std::cout<<"radiation_num_divider["<<i<<"] = "<<radiation_num_divider[i]<<std::endl;
    }


    for(int i=0;i<range_max.size();i++)
    {
        angle_resolution.push_back(GRID_SPACING/range_max[i]);
    }

    for(int i=0;i<angle_resolution.size();i++)
    {
        radiation_num.push_back(static_cast<uint32_t>(2*M_PI/angle_resolution[i]));
    }


    

    float current_grid[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
            current_grid[i][j]=l[probability::unknown];

    float history_grid[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
            history_grid[i][j]=l[probability::unknown];

    uint8_t pcl_grid[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
            pcl_grid[i][j]=discrete_pcl::unset;

    float (*current_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&current_grid;
    float (*history_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&history_grid;
    uint8_t (*pcl_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&pcl_grid;



    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh,"/VehiclePoseFusionUTM",1000);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZL>> pcl_sub(nh,"/as_tx/point_cloud",1000);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, pcl::PointCloud<pcl::PointXYZL>> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose_sub,pcl_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,pcl_grid_pointer,ptracs));

    ros::Publisher car_pub = nh.advertise<nav_msgs::GridCells>("car_visual",1000);
    ros::Publisher anchor_pub = nh.advertise<nav_msgs::GridCells>("acs_visual",1000);
    ros::Publisher oG_pub = nh.advertise<nav_msgs::OccupancyGrid>("oG",1000);
    
    //Wait a valid msg and then initialize the anchor position
    ROS_INFO("Waiting message... ...");
    while(ros::ok())
    {
        ros::spinOnce();
        //ROS_INFO("(x,y,yaw)=( %f, %f, %f )",car.get_x(),car.get_y(),car.get_psi());
        if (car.get_x()!=0.0)
        {
            break;
        }        
        loop_rate.sleep();
    }
    ROS_INFO("Message arrived!");
    anchor_init(car,acs);
    ROS_INFO("Car position ( %f, %f ) ",car.get_x(),car.get_y());
    ROS_INFO("Anchor init_postion ( %f, %f )",acs.get_current_anchor_x(),acs.get_current_anchor_y());
    ROS_INFO("difference ( %f, %f ) ",car.get_x()-acs.get_current_anchor_x(),car.get_y()-acs.get_current_anchor_y());
    
    
    while(ros::ok())
    {
        ros::spinOnce();//call all the callbacks waiting to be called at that point in time.
         
        if(if_get_new_data==true)
        {       
            geometry_msgs::Pose og_origin;
            og_origin.position.x=acs.get_current_anchor_x()-VISUAL_OFFSET_X;
            og_origin.position.y=acs.get_current_anchor_y()-VISUAL_OFFSET_Y;
            og_origin.position.z=0.0;  
            oG2D oG2D(0.1,GRID_SIZE_X,GRID_SIZE_Y,og_origin);
            geometry_msgs::Point cluster_point; //One point to be used as argument for all vectors push back

            //anchor++++
            std::vector<geometry_msgs::Point> anchor_cluster;
            //cluster_point.x=ACS::anchor_x;
            //cluster_point.y=ACS::anchor_y;
            cluster_point.x=acs.get_current_anchor_x();
            cluster_point.y=acs.get_current_anchor_y();
            cluster_point.z=0.0;   
            anchor_cluster.push_back(cluster_point);
           
            //car++++
            std::vector<geometry_msgs::Point> car_grid;
            generate_car_cluster(car.get_x(),car.get_y(),car.get_psi(),acs.get_current_anchor_x(),acs.get_current_anchor_y(),car_grid);
            
            for(int i=0;i<car_grid.size();i++)
            {
                current_grid[uint16_t(car_grid[i].x)][uint16_t(car_grid[i].y)]=l[probability::clear];
            }
        
            std::vector<geometry_msgs::Point> car_cluster;
            for(int i=0;i<car_grid.size();i++)
            {
                //cluster_point.x=car_grid[i].x*GRID_SPACING+ACS::anchor_x;
                //cluster_point.y=car_grid[i].y*GRID_SPACING+ACS::anchor_y;
                cluster_point.x=car_grid[i].x*GRID_SPACING+acs.get_current_anchor_x();
                cluster_point.y=car_grid[i].y*GRID_SPACING+acs.get_current_anchor_y();
                cluster_point.z=0.0; 
                car_cluster.push_back(cluster_point);
            }

        
            
            
            

            
            update_grid_when_anchor_moved(history_grid_pointer,l[probability::unknown],acs);
            create_freespace(pcl_grid_pointer,current_grid_pointer,l,acs);
            binary_bayes(current_grid_pointer,history_grid_pointer,l[probability::max],l[probability::min],l[probability::unknown]);
            

            std::vector<int8_t> probability_vector;

            
            for(auto j=0;j<GRID_SIZE_Y;j++)
                for(auto i=0;i<GRID_SIZE_X;i++)
                {
                    if(history_grid[i][j]==l[probability::unknown])
                    {
                        probability_vector.push_back(-1);
                    }
                    else 
                    {
                        int8_t p=100.0-(100.0/(1.0+exp(history_grid[i][j])));
                        probability_vector.push_back(p);    
                    }
                    
                }

        
            
            oG2D.visualization(probability_vector,oG_pub,"map");
            


            visualize_girdcells(anchor_cluster,"map",float(1),float(1),anchor_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
            visualize_girdcells(car_cluster,"map",float(GRID_SPACING),float(GRID_SPACING),car_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
        
        
            for(int i=0;i<GRID_SIZE_X;i++)
                for(int j=0;j<GRID_SIZE_Y;j++)
                {
                    current_grid[i][j]=l[probability::unknown];
                }

            acs.set_history_anchor_x(acs.get_current_anchor_x());
            acs.set_history_anchor_y(acs.get_current_anchor_y());
             
            
            if_get_new_data=false;
            
        }
        

    }
     



}

void update_grid_when_anchor_moved(float (*grid)[GRID_SIZE_X][GRID_SIZE_Y],float new_are_fullfill,const ABC& acs)
{
    if((acs.get_current_anchor_x()==acs.get_history_anchor_x())&&(acs.get_current_anchor_y()==acs.get_history_anchor_y()))
    {
        return;
    }
    uint16_t grid_trans_x=std::abs((acs.get_current_anchor_x()-acs.get_history_anchor_x())/GRID_SPACING);
    uint16_t grid_trans_y=std::abs((acs.get_current_anchor_y()-acs.get_history_anchor_y())/GRID_SPACING);
    ROS_INFO("grird_trans_x %d",grid_trans_x);
    ROS_INFO("grird_trans_y %d",grid_trans_y);
          
    if(acs.get_current_anchor_x()<acs.get_history_anchor_x())
    {
        ROS_INFO("anchor moved left");
        for(int16_t i=GRID_SIZE_X-1;i>=grid_trans_x;i--)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=(*grid)[i-grid_trans_x][j];
            }
        for(int16_t i=grid_trans_x-1;i>=0;i--)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=new_are_fullfill;
            }
    }
    else if(acs.get_current_anchor_x()>acs.get_history_anchor_x())
    {
        ROS_INFO("anchor moved right");
        for(int16_t i=0;i<=GRID_SIZE_X-1-grid_trans_x;i++)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=(*grid)[i+grid_trans_x][j];
            }
        for(int16_t i=GRID_SIZE_X-grid_trans_x;i<=GRID_SIZE_X-1;i++)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=new_are_fullfill;
            }
    }
    if(acs.get_current_anchor_y()<acs.get_history_anchor_y())
    {
        ROS_INFO("anchor moved down");
        
        for(int16_t i=0;i<=GRID_SIZE_X-1;i++)
            for(int16_t j=GRID_SIZE_Y-1;j>=grid_trans_y;j--)
            {
                (*grid)[i][j]=(*grid)[i][j-grid_trans_y];
            }
        for(int16_t i=0;i<=GRID_SIZE_X-1;i++)
            for(int16_t j=grid_trans_y-1;j>=0;j--)
            {
                (*grid)[i][j]=new_are_fullfill;
            }
    }
    else if(acs.get_current_anchor_y()>acs.get_history_anchor_y())
    {
        ROS_INFO("anchor moved up");
        for(int16_t i=0;i<GRID_SIZE_X;i++)
            for(int16_t j=0;j<=GRID_SIZE_Y-1-grid_trans_y;j++)
            {
                (*grid)[i][j]=(*grid)[i][j+grid_trans_y];
            }
        for(int16_t i=0;i<GRID_SIZE_X;i++)
            for(int16_t j=GRID_SIZE_Y-grid_trans_y;j<=GRID_SIZE_Y-1;j++)
            {
                (*grid)[i][j]=new_are_fullfill;
            }
    }
}
/*
void update_grid_when_anchor_moved(float (*grid)[GRID_SIZE_X][GRID_SIZE_Y],float new_are_fullfill,const ABC& acs)
{
    if((acs.get_current_anchor_x()==acs.get_history_anchor_x())&&(acs.get_current_anchor_y()==acs.get_history_anchor_y()))
    {
        return;
    }
    uint16_t grid_trans_x=std::abs((acs.get_current_anchor_x()-acs.get_history_anchor_x())/GRID_SPACING);
    uint16_t grid_trans_y=std::abs((acs.get_current_anchor_y()-acs.get_history_anchor_y())/GRID_SPACING);
    ROS_INFO("grird_trans_x %d",grid_trans_x);
    ROS_INFO("grird_trans_y %d",grid_trans_y);
    if(acs.get_current_anchor_x()<acs.get_history_anchor_x())
    {
        ROS_INFO("anchor moved left");
        for(int16_t i=acs.get_grid_trans_x_cell()-1;i>=0;i--)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=new_are_fullfill;
            }
        for(int16_t i=GRID_SIZE_X-1;i>=acs.get_grid_trans_x_cell();i--)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=(*grid)[i-acs.get_grid_trans_x_cell()][j];
            }
    }
    else if(acs.get_current_anchor_x()>acs.get_history_anchor_x())
    {
        ROS_INFO("anchor moved right");
        for(int16_t i=GRID_SIZE_X-acs.get_grid_trans_x_cell();i<=GRID_SIZE_X-1;i++)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=new_are_fullfill;
            }
        for(int16_t i=0;i<=GRID_SIZE_X-1-acs.get_grid_trans_x_cell();i++)
            for(int16_t j=0;j<GRID_SIZE_Y;j++)
            {
                (*grid)[i][j]=(*grid)[i+acs.get_grid_trans_x_cell()][j];
            }
    }
    if(acs.get_current_anchor_y()<acs.get_history_anchor_y())
    {
        ROS_INFO("anchor moved down");
        for(int16_t i=0;i<GRID_SIZE_X;i++)
            for(int16_t j=acs.get_grid_trans_y_cell()-1;j>=0;j--)
            {
                (*grid)[i][j]=new_are_fullfill;
            }
        for(int16_t i=0;i<GRID_SIZE_X;i++)
            for(int16_t j=GRID_SIZE_Y-1;j>=acs.get_grid_trans_y_cell();j--)
            {
                (*grid)[i][j]=(*grid)[i][j-acs.get_grid_trans_y_cell()];
            }
    }
    else if(acs.get_current_anchor_y()>acs.get_history_anchor_y())
    {
        ROS_INFO("anchor moved up");
        for(int16_t i=0;i<GRID_SIZE_X;i++)
            for(int16_t j=GRID_SIZE_Y-acs.get_grid_trans_y_cell();j<=GRID_SIZE_Y-1;j++)
            {
                (*grid)[i][j]=new_are_fullfill;
            }

        for(int16_t i=0;i<GRID_SIZE_X;i++)
            for(int16_t j=0;j<=GRID_SIZE_Y-1-acs.get_grid_trans_y_cell();j++)
            {
                (*grid)[i][j]=(*grid)[i][j+acs.get_grid_trans_y_cell()];
            }
    }
}
 */



void anchor_init(const Vehicle& car,ABC& acs)
{
    //Make the anchor near the car so that it could quickly determine its initilazied position and make sure it is 10*Z number
    acs.set_history_anchor_x(double(static_cast<int64_t>(car.get_x())/10)*10.0);
    acs.set_history_anchor_y(double(static_cast<int64_t>(car.get_y())/10)*10.0);
    while(  (car.get_x()<= acs.get_current_anchor_x()+acs.get_borderoFcar_x1_m())||
            (car.get_x()>=acs.get_current_anchor_x()+acs.get_borderoFcar_x2_m())||
            (car.get_y()<=acs.get_current_anchor_y()+acs.get_borderoFcar_y1_m())||
            (car.get_y()>=acs.get_current_anchor_y()+acs.get_borderoFcar_y2_m())
        )
    {
        adjust_ACS(acs);
    }

    //ACS::history_anchor_x=ACS::anchor_x;
    //ACS::history_anchor_y=ACS::anchor_y;
    acs.set_history_anchor_x(acs.get_current_anchor_x());
    acs.set_history_anchor_y(acs.get_current_anchor_y());

}



void adjust_ACS(ABC &acs)
{
    //adjust the anchor of ACS if it is necessary
    if (car.get_x()<(acs.get_current_anchor_x()+acs.get_borderoFcar_x1_m()))
    {
        acs.set_current_anchor_x(acs.get_current_anchor_x()-acs.get_grid_trans_x_m());
    }
    else if (car.get_x()>(acs.get_current_anchor_x()+acs.get_borderoFcar_x2_m()))
    {
        acs.set_current_anchor_x(acs.get_current_anchor_x()+acs.get_grid_trans_x_m());
    }
    if(car.get_y()<(acs.get_current_anchor_y()+acs.get_borderoFcar_y1_m()))
    {
        acs.set_current_anchor_y(acs.get_current_anchor_y()-acs.get_grid_trans_y_m());
    }
    else if(car.get_y()>(acs.get_current_anchor_y()+acs.get_borderoFcar_y2_m()))
    {
        acs.set_current_anchor_y(acs.get_current_anchor_y()+acs.get_grid_trans_y_m());
    }
}

void adjust_ACS(ABC* acs)
{
    //adjust the anchor of ACS if it is necessary
    if (car.get_x()<(acs->get_current_anchor_x()+acs->get_borderoFcar_x1_m()))
    {
        acs->set_current_anchor_x(acs->get_current_anchor_x()-acs->get_grid_trans_x_m());
    }
    else if (car.get_x()>(acs->get_current_anchor_x()+acs->get_borderoFcar_x2_m()))
    {
        acs->set_current_anchor_x(acs->get_current_anchor_x()+acs->get_grid_trans_x_m());
    }
    if(car.get_y()<(acs->get_current_anchor_y()+acs->get_borderoFcar_y1_m()))
    {
        acs->set_current_anchor_y(acs->get_current_anchor_y()-acs->get_grid_trans_y_m());
    }
    
    else if(car.get_y()>(acs->get_current_anchor_y()+acs->get_borderoFcar_y2_m()))
    {
        acs->set_current_anchor_y(acs->get_current_anchor_y()+acs->get_grid_trans_y_m());
    }
}


 void callback(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg,const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& pcl_msg,uint8_t(*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],ABC* ptracs)
{
    uint64_t pose_ts_ns=uint64_t(pose_msg->header.stamp.nsec)/1000; //pose time stamp nano second
    uint64_t pose_ts_s=uint64_t(pose_msg->header.stamp.sec);        //pose time stamp second
    uint64_t stamp_pose=1000000*pose_ts_s+pose_ts_ns;

    uint64_t stamp_pcl=pcl_msg->header.stamp;
    uint64_t pcl_ts_ns=stamp_pcl%1000000;
    uint64_t pcl_ts_s=stamp_pcl/1000000;

    int64_t diff=stamp_pose-stamp_pcl;
    
    if(abs(diff)<allowed_diff_ts_ns)//Data is valid
    {
        bool skip_the_frame {false};//Flag to check if we should skip the frame

        BOOST_FOREACH(const pcl::PointXYZL& pt, pcl_msg->points)
        {
          if(pt.label>=4)
          {
            skip_the_frame=true;
            break;
          }  
        }
        if(skip_the_frame==false)
        {
            
            car.set_x(pose_msg->pose.position.x);
            car.set_y(pose_msg->pose.position.y);
            car.set_psi(pose_msg->pose.position.z); 

            adjust_ACS(ptracs);

            for(int i=0;i<GRID_SIZE_X;i++)
                for(int j=0;j<GRID_SIZE_Y;j++)
                {
                    (*pcl_grid)[i][j]=discrete_pcl::unset;
                } 
            BOOST_FOREACH (const pcl::PointXYZL& pt, pcl_msg->points)
            {
                if((pt.label!=2)&&(pt.label!=3)&&(pt.label!=1)&&(pt.label!=0))
                {
                    continue;
                }
                if(pt.label==0)
                {
                    if(pt.x>0)
                    {   
                        if((pt.z<2.8)&&((pt.z-0.1)>0))
                        {
                            geometry_msgs::Point pcl_acs;
                            double pos_psy=(-car.get_psi()+90)/180*M_PI;
                            pcl_acs.x=cos(pos_psy)*pt.x-sin(pos_psy)*pt.y;
                            pcl_acs.y=sin(pos_psy)*pt.x+cos(pos_psy)*pt.y;
                            pcl_acs.x=pcl_acs.x+car.get_x()-ptracs->get_current_anchor_x();
                            pcl_acs.y=pcl_acs.y+car.get_y()-ptracs->get_current_anchor_y();
                            if( (pcl_acs.x>=0) && (pcl_acs.x<=double(GRID_SIZE_X*GRID_SPACING)) && (pcl_acs.y>=0) && (pcl_acs.y<=double(GRID_SIZE_Y*GRID_SPACING)) )
                            {
                                uint16_t grid_x=uint16_t(floor(pcl_acs.x/GRID_SPACING));
                                uint16_t grid_y=uint16_t(floor(pcl_acs.y/GRID_SPACING));
                                (*pcl_grid)[grid_x][grid_y]=discrete_pcl::set;
                            } 
                        }
                    }
                     
                }
                else
                {
                    if((pt.z<2.8)&&((pt.z-0.1)>0))
                    {
                        geometry_msgs::Point pcl_acs;
                        double pos_psy=(-car.get_psi()+90)/180*M_PI;
                        pcl_acs.x=cos(pos_psy)*pt.x-sin(pos_psy)*pt.y;
                        pcl_acs.y=sin(pos_psy)*pt.x+cos(pos_psy)*pt.y;
                        pcl_acs.x=pcl_acs.x+car.get_x()-ptracs->get_current_anchor_x();
                        pcl_acs.y=pcl_acs.y+car.get_y()-ptracs->get_current_anchor_y();
                        if( (pcl_acs.x>=0) && (pcl_acs.x<=double(GRID_SIZE_X*GRID_SPACING)) && (pcl_acs.y>=0) && (pcl_acs.y<=double(GRID_SIZE_Y*GRID_SPACING)) )
                        {
                            uint16_t grid_x=uint16_t(floor(pcl_acs.x/GRID_SPACING));
                            uint16_t grid_y=uint16_t(floor(pcl_acs.y/GRID_SPACING));
                            (*pcl_grid)[grid_x][grid_y]=discrete_pcl::set;
                            flag_msg_valid=true;
                        } 
                    }
                }
                
                
            }
             
            if(flag_msg_valid==false)
            {
                ROS_INFO("MSG NOT VALID!");
            }
            else if(flag_msg_valid==true)
            {
                flag_msg_valid=false;
                if_get_new_data=true;
            }

            
        }
        
          
        
          
        
    }
    
     
}







void create_freespace(uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],const std::vector<float>& l,const ABC& acs)
{
    double offset_x=car.get_x()-acs.get_current_anchor_x();
    double offset_y=car.get_y()-acs.get_current_anchor_y();

    for(int laser_num=0;laser_num<laser_x.size();laser_num++)
    {
        double pos_psy=(-car.get_psi()+90)/180*M_PI;
        double laser_abs_x=cos(pos_psy)*laser_x[laser_num]-sin(pos_psy)*laser_y[laser_num]+offset_x;
        double laser_abs_y=sin(pos_psy)*laser_x[laser_num]+cos(pos_psy)*laser_y[laser_num]+offset_y;

        double laser_abs_yaw=std::fmod(pos_psy+laser_yaw[laser_num],2*M_PI);

        int32_t xstart=floor(laser_abs_x/GRID_SPACING);
        int32_t ystart=floor(laser_abs_y/GRID_SPACING);

        for(int num=0;num<(radiation_num[laser_num]/radiation_num_divider[laser_num]);num++)
        {
            int32_t beam_x=floor(cos(angle_resolution[laser_num]*num+laser_abs_yaw)*range_max[laser_num]/GRID_SPACING);
            int32_t beam_y=floor(sin(angle_resolution[laser_num]*num+laser_abs_yaw)*range_max[laser_num]/GRID_SPACING);

            int32_t xend=xstart+beam_x;
            int32_t yend=ystart+beam_y;
            
            if(xend>(GRID_SIZE_X-1))
            {
                beam_x=GRID_SIZE_X-1-xstart;
                double dmaxtemp=std::abs(beam_x*GRID_SPACING/(cos(angle_resolution[laser_num]*num+laser_abs_yaw)));

                beam_y=floor(sin(angle_resolution[laser_num]*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                xend=GRID_SIZE_X-1;
                yend=ystart+beam_y;
                
            }
            else if(xend<0)
            {
                beam_x=xstart-0;
                double dmaxtemp=std::abs(beam_x*GRID_SPACING/(cos(angle_resolution[laser_num]*num+laser_abs_yaw)));

                beam_y=floor(sin(angle_resolution[laser_num]*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                xend=0;
                yend=ystart+beam_y;
            }

            if(yend>(GRID_SIZE_Y-1))
            {
                beam_y=GRID_SIZE_Y-1-ystart;
                double dmaxtemp=std::abs(beam_y*GRID_SPACING/(sin(angle_resolution[laser_num]*num+laser_abs_yaw)));
                
                beam_x=floor(cos(angle_resolution[laser_num]*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                yend=GRID_SIZE_X-1;
                xend=xstart+beam_x;
            }
            else if(yend<0)
            {
                beam_y=ystart-0;
                double dmaxtemp=std::abs(beam_y*GRID_SPACING/(sin(angle_resolution[laser_num]*num+laser_abs_yaw)));

                beam_y=floor(cos(angle_resolution[laser_num]*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                yend=0;
                xend=xstart+beam_x;
            }
        
            int32_t dx=round(xend-xstart);
            int32_t dy=round(yend-ystart);

    
            int8_t incx=sign(dx);
            int8_t incy=sign(dy);
            
            int pdx;
            int pdy;
            int ddx;
            int ddy;
            int32_t deltaslowdirection=0;
            int32_t deltafastdirection=0;
            if(dx<0)
                {dx=-dx;}
            if(dy<0)
                {dy=-dy;}

            if(dx>dy)
            {
                pdx=incx;
                pdy=0;
                ddx=incx;
                ddy=incy;

                deltaslowdirection=dy;
                deltafastdirection=dx;
            }
            else if(dx<=dy)
            {
                pdx=0;
                pdy=incy;
                ddx=incx;
                ddy=incy;

                deltaslowdirection=dx;
                deltafastdirection=dy;
            }
        
        
            int32_t x=xstart;
            int32_t y=ystart;
            int32_t err=deltafastdirection/2;


            bool if_obstacle {false};

            
            for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
            {    
                err=err-deltaslowdirection;
                if(err<0)
                {
                    err=err+deltafastdirection;
                    x=x+ddx;
                    y=y+ddy;
                }
                else
                {
                    x=x+pdx;
                    y=y+pdy;
                }

                double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                
                if((*pcl_grid)[uint16_t(x)][uint16_t(y)]==discrete_pcl::set)
                {
                
                    if((d<range_max[laser_num])&&(d>=range_min[laser_num]))
                    {
                        (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::fill];
                        if_obstacle=true;
                        break;
                    }

                }
                
            
            }

            if(if_obstacle==true)
            {
                x=xstart;
                y=ystart;
                err=deltafastdirection/2;
                for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
                {    
                    err=err-deltaslowdirection;
                    if(err<0)
                    {
                        err=err+deltafastdirection;
                        x=x+ddx;
                        y=y+ddy;
                    }
                    else
                    {
                        x=x+pdx;
                        y=y+pdy;
                    }

                    if((*current_grid)[uint16_t(x)][uint16_t(y)]==l[probability::fill])
                    {
                        break;
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]==l[probability::unknown])
                    {
                        double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                        if (d<range_min[laser_num])
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                        }
                        else if((d<range_max[laser_num])&&(d>=range_min[laser_num]))
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                            //(*current_grid)[uint16_t(x)][uint16_t(y)]=20+0.5*(d-DMIN);
                        }
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]!=l[probability::unknown])
                    {continue;}
                    
                
                }
            }

            if(if_obstacle==false)
            {
                x=xstart;
                y=ystart;
                err=deltafastdirection/2;
                for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
                {    
                    err=err-deltaslowdirection;
                    if(err<0)
                    {
                        err=err+deltafastdirection;
                        x=x+ddx;
                        y=y+ddy;
                    }
                    else
                    {
                        x=x+pdx;
                        y=y+pdy;
                    }
                    double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                    if(d<range_max[laser_num])
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                        }
                    
                    

                    
                
                }
            }
            
        }
    }

} 



/*
void create_freespace(uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],const std::vector<float>& l,const ABC& acs)
{
    double offset_x=car.get_x()-acs.get_current_anchor_x();
    double offset_y=car.get_y()-acs.get_current_anchor_y();

    for(int laser_num=0;laser_num<laser_x.size();laser_num++)
    {
        double pos_psy=(-car.get_psi()+90)/180*M_PI;
        double laser_abs_x=cos(pos_psy)*laser_x[laser_num]-sin(pos_psy)*laser_y[laser_num]+offset_x;
        double laser_abs_y=sin(pos_psy)*laser_x[laser_num]+cos(pos_psy)*laser_y[laser_num]+offset_y;

        double laser_abs_yaw=std::fmod(pos_psy+laser_yaw[laser_num],2*M_PI);

        int32_t xstart=floor(laser_abs_x/GRID_SPACING);
        int32_t ystart=floor(laser_abs_y/GRID_SPACING);

        for(int num=0;num<(radiation_num/radiation_num_divder);num++)
        {
            int32_t beam_x=floor(cos(angle_resolution*num+laser_abs_yaw)*DMAX/GRID_SPACING);
            int32_t beam_y=floor(sin(angle_resolution*num+laser_abs_yaw)*DMAX/GRID_SPACING);

            int32_t xend=xstart+beam_x;
            int32_t yend=ystart+beam_y;
            
            if(xend>(GRID_SIZE_X-1))
            {
                beam_x=GRID_SIZE_X-1-xstart;
                double dmaxtemp=std::abs(beam_x*GRID_SPACING/(cos(angle_resolution*num+laser_abs_yaw)));

                beam_y=floor(sin(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                xend=GRID_SIZE_X-1;
                yend=ystart+beam_y;
                
            }
            else if(xend<0)
            {
                beam_x=xstart-0;
                double dmaxtemp=std::abs(beam_x*GRID_SPACING/(cos(angle_resolution*num+laser_abs_yaw)));

                beam_y=floor(sin(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                xend=0;
                yend=ystart+beam_y;
            }

            if(yend>(GRID_SIZE_Y-1))
            {
                beam_y=GRID_SIZE_Y-1-ystart;
                double dmaxtemp=std::abs(beam_y*GRID_SPACING/(sin(angle_resolution*num+laser_abs_yaw)));
                
                beam_x=floor(cos(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                yend=GRID_SIZE_X-1;
                xend=xstart+beam_x;
            }
            else if(yend<0)
            {
                beam_y=ystart-0;
                double dmaxtemp=std::abs(beam_y*GRID_SPACING/(sin(angle_resolution*num+laser_abs_yaw)));

                beam_y=floor(cos(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                yend=0;
                xend=xstart+beam_x;
            }
        
            int32_t dx=round(xend-xstart);
            int32_t dy=round(yend-ystart);

    
            int8_t incx=sign(dx);
            int8_t incy=sign(dy);
            
            int pdx;
            int pdy;
            int ddx;
            int ddy;
            int32_t deltaslowdirection=0;
            int32_t deltafastdirection=0;
            if(dx<0)
                {dx=-dx;}
            if(dy<0)
                {dy=-dy;}

            if(dx>dy)
            {
                pdx=incx;
                pdy=0;
                ddx=incx;
                ddy=incy;

                deltaslowdirection=dy;
                deltafastdirection=dx;
            }
            else if(dx<=dy)
            {
                pdx=0;
                pdy=incy;
                ddx=incx;
                ddy=incy;

                deltaslowdirection=dx;
                deltafastdirection=dy;
            }
        
        
            int32_t x=xstart;
            int32_t y=ystart;
            int32_t err=deltafastdirection/2;


            bool if_obstacle {false};

            
            for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
            {    
                err=err-deltaslowdirection;
                if(err<0)
                {
                    err=err+deltafastdirection;
                    x=x+ddx;
                    y=y+ddy;
                }
                else
                {
                    x=x+pdx;
                    y=y+pdy;
                }

                double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                
                if((*pcl_grid)[uint16_t(x)][uint16_t(y)]==discrete_pcl::set)
                {
                
                    if((d<DMAX)&&(d>=DMIN))
                    {
                        (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::fill];
                        if_obstacle=true;
                        break;
                    }

                }
                
            
            }

            if(if_obstacle==true)
            {
                x=xstart;
                y=ystart;
                err=deltafastdirection/2;
                for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
                {    
                    err=err-deltaslowdirection;
                    if(err<0)
                    {
                        err=err+deltafastdirection;
                        x=x+ddx;
                        y=y+ddy;
                    }
                    else
                    {
                        x=x+pdx;
                        y=y+pdy;
                    }

                    if((*current_grid)[uint16_t(x)][uint16_t(y)]==l[probability::fill])
                    {
                        break;
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]==l[probability::unknown])
                    {
                        double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                        if (d<DMIN)
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                        }
                        else if((d<DMAX)&&(d>=DMIN))
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                            //(*current_grid)[uint16_t(x)][uint16_t(y)]=20+0.5*(d-DMIN);
                        }
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]!=l[probability::unknown])
                    {continue;}
                    
                
                }
            }

            if(if_obstacle==false)
            {
                x=xstart;
                y=ystart;
                err=deltafastdirection/2;
                for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
                {    
                    err=err-deltaslowdirection;
                    if(err<0)
                    {
                        err=err+deltafastdirection;
                        x=x+ddx;
                        y=y+ddy;
                    }
                    else
                    {
                        x=x+pdx;
                        y=y+pdy;
                    }
                    double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                    if(d<DMAX)
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                        }
                    
                    

                    
                
                }
            }
            
        }
    }
} 
*/




/* 
void create_freespace(uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],const std::vector<float>& l,const ABC& acs)
{
    double offset_x=car.get_x()-acs.get_current_anchor_x();
    double offset_y=car.get_y()-acs.get_current_anchor_y();

    for(int laser_num=0;laser_num<laser_x.size();laser_num++)
    {
        double pos_psy=(-car.get_psi()+90)/180*M_PI;
        double laser_abs_x=cos(pos_psy)*laser_x[laser_num]-sin(pos_psy)*laser_y[laser_num]+offset_x;
        double laser_abs_y=sin(pos_psy)*laser_x[laser_num]+cos(pos_psy)*laser_y[laser_num]+offset_y;

        double laser_abs_yaw=std::fmod(pos_psy+laser_yaw[laser_num],2*M_PI);

        int32_t xstart=floor(laser_abs_x/GRID_SPACING);
        int32_t ystart=floor(laser_abs_y/GRID_SPACING);

        double llrd=DMAX; //last_laser_reach_distance
        bool detect_lasttime {false};

        for(int num=0;num<(radiation_num/radiation_num_divder);num++)
        {
            int32_t beam_x=floor(cos(angle_resolution*num+laser_abs_yaw)*DMAX/GRID_SPACING);
            int32_t beam_y=floor(sin(angle_resolution*num+laser_abs_yaw)*DMAX/GRID_SPACING);

            int32_t xend=xstart+beam_x;
            int32_t yend=ystart+beam_y;
            
            if(xend>(GRID_SIZE_X-1))
            {
                beam_x=GRID_SIZE_X-1-xstart;
                double dmaxtemp=std::abs(beam_x*GRID_SPACING/(cos(angle_resolution*num+laser_abs_yaw)));

                beam_y=floor(sin(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                xend=GRID_SIZE_X-1;
                yend=ystart+beam_y;
                
            }
            else if(xend<0)
            {
                beam_x=xstart-0;
                double dmaxtemp=std::abs(beam_x*GRID_SPACING/(cos(angle_resolution*num+laser_abs_yaw)));

                beam_y=floor(sin(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                xend=0;
                yend=ystart+beam_y;
            }

            if(yend>(GRID_SIZE_Y-1))
            {
                beam_y=GRID_SIZE_Y-1-ystart;
                double dmaxtemp=std::abs(beam_y*GRID_SPACING/(sin(angle_resolution*num+laser_abs_yaw)));
                
                beam_x=floor(cos(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                yend=GRID_SIZE_X-1;
                xend=xstart+beam_x;
            }
            else if(yend<0)
            {
                beam_y=ystart-0;
                double dmaxtemp=std::abs(beam_y*GRID_SPACING/(sin(angle_resolution*num+laser_abs_yaw)));

                beam_y=floor(cos(angle_resolution*num+laser_abs_yaw)*dmaxtemp/GRID_SPACING);

                yend=0;
                xend=xstart+beam_x;
            }
        
            int32_t dx=round(xend-xstart);
            int32_t dy=round(yend-ystart);

    
            int8_t incx=sign(dx);
            int8_t incy=sign(dy);
            
            int pdx;
            int pdy;
            int ddx;
            int ddy;
            int32_t deltaslowdirection=0;
            int32_t deltafastdirection=0;
            if(dx<0)
                {dx=-dx;}
            if(dy<0)
                {dy=-dy;}

            if(dx>dy)
            {
                pdx=incx;
                pdy=0;
                ddx=incx;
                ddy=incy;

                deltaslowdirection=dy;
                deltafastdirection=dx;
            }
            else if(dx<=dy)
            {
                pdx=0;
                pdy=incy;
                ddx=incx;
                ddy=incy;

                deltaslowdirection=dx;
                deltafastdirection=dy;
            }
        
        
            int32_t x=xstart;
            int32_t y=ystart;
            int32_t err=deltafastdirection/2;


            bool if_obstacle {false};

            
            for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
            {    
                err=err-deltaslowdirection;
                if(err<0)
                {
                    err=err+deltafastdirection;
                    x=x+ddx;
                    y=y+ddy;
                }
                else
                {
                    x=x+pdx;
                    y=y+pdy;
                }

                double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                
                if((*pcl_grid)[uint16_t(x)][uint16_t(y)]==discrete_pcl::set)
                {
                
                    if((d<DMAX)&&(d>=DMIN))
                    {
                        (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::fill];
                        if_obstacle=true;
                        llrd=d;
                        break;
                    }

                }
                
            
            }

            if(if_obstacle==true)
            {
                x=xstart;
                y=ystart;
                err=deltafastdirection/2;
                for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
                {    
                    err=err-deltaslowdirection;
                    if(err<0)
                    {
                        err=err+deltafastdirection;
                        x=x+ddx;
                        y=y+ddy;
                    }
                    else
                    {
                        x=x+pdx;
                        y=y+pdy;
                    }

                    if((*current_grid)[uint16_t(x)][uint16_t(y)]==l[probability::fill])
                    {
                        break;
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]==l[probability::unknown])
                    {
                        double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                        if (d<DMIN)
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                        }
                        else if((d<DMAX)&&(d>=DMIN))
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                            //(*current_grid)[uint16_t(x)][uint16_t(y)]=20+0.5*(d-DMIN);
                        }
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]!=l[probability::unknown])
                    {continue;}
                    
                
                }
                detect_lasttime=true;
            }

            if(if_obstacle==false)
            {
                x=xstart;
                y=ystart;
                err=deltafastdirection/2;
                for(int32_t j=0;j<(deltafastdirection);j++)//Check if i have the obstabcle
                {    
                    err=err-deltaslowdirection;
                    if(err<0)
                    {
                        err=err+deltafastdirection;
                        x=x+ddx;
                        y=y+ddy;
                    }
                    else
                    {
                        x=x+pdx;
                        y=y+pdy;
                    }

                    if(detect_lasttime==true)
                    {
                        double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                        if(d<llrd)
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                        }
                    }
                    else if(detect_lasttime==false)
                    {
                        double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                        if(d<DMAX)
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l[probability::clear];
                        }
                    }
                    

                    
                
                }
                detect_lasttime=false;
            }
            
        }
    }
}

*/




