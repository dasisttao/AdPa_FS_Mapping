#include "grid_based_mapping_tao/parameter_grid.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
/*++++++++++++++++++++++++++This node is doing the following  jobs+++++++++++++++++++++++++++++++++++++++++
    
        Get Message from Topic /VehiclePoseFusionUTM so that we can get the car pose 
        Get Message from Topic /as_tx/point_cloud so that we can get the pcl message

      Publish the Message to Topic /data2visual so that we can visualize it with rviz.The Message conatains
      car cluster, anchor position and occupancygrid(history_grid)(by a varibale of type std_msgs/UInt8MultiArray)


 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

enum State {Grid_occupied=255,Grid_free=254,Grid_car=253};

//++++++++++++++++++++++++++++++++++++++++++++++++global varibale+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Vehicle car;//create global object car 

const double epsilon =1;
const uint8_t binary_threshold {50};

uint64_t allowed_diff_ts_ns=25000;//0.025s


std::vector<geometry_msgs::Point> test_cluster;

uint8_t test_grid[GRID_SIZE_X][GRID_SIZE_Y];
  


std::vector<geometry_msgs::Point> obj_cluster_temp;
//++++++++++++++++++++++++++++++++++++++++++++++++++Functions++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

template <size_t row, size_t col>
void print_out_grid(uint8_t (&array)[row][col]);
void adj(uint8_t (*grid)[GRID_SIZE_X][GRID_SIZE_Y]);
void callback(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg,const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& pcl_msg,uint8_t (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y]);
void anchor_init(const Vehicle& car);
void adjust_ACS();
void generate_car_cluster(double ego_x,double ego_y,double ego_yaw,double anchor_x,double ancor_y,std::vector<geometry_msgs::Point> &car_grid);
std::vector<double> linspace(double a,double b, int n);
void Bayes(uint8_t (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y]);

void freespace(uint8_t (*array)[GRID_SIZE_X][GRID_SIZE_Y]);
void freespace_debug(uint8_t (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y]);

double sind(double value_in_degree);
double cosd(double value_in_degree);
int sign(int x);
//visualize Cells
void visualize_girdcells(const std::vector<geometry_msgs::Point>& cells,const std::string& frame_id,float grid_cell_width,float grid_cell_height,
                        ros::Publisher pub,double offset_x,double offset_y);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++RAY CASTING+++++++++++++++++++++++++++++++++++
namespace TEASY_LASERS
{
  double DMAX=10; //max distannce of reachement
  double DMIN=5;
  std::vector<double> laser_x {+3.6,-1.0,+3.3,+3.3,-0.5,-0.5}; //In VCS Vehicle Coordinate System
  std::vector<double> laser_y {+0.0,+0.0,-0.8,+0.8,-0.8,+0.8};
  std::vector<double> laser_yaw {-45*M_PI/180,-225*M_PI/180,-125*M_PI/180,35*M_PI/180,-145*M_PI/180,55*M_PI/180};

  //std::vector<double> laser_x {+3.6}; //In VCS Vehicle Coordinate System
  //std::vector<double> laser_y {+0.0};
  //std::vector<double> laser_yaw {-45*M_PI/180};

  double angle_resolution=GRID_SPACING/DMAX;
  uint32_t radiation_num=static_cast<uint32_t>(2*M_PI/angle_resolution);
  uint8_t radiation_num_divder=4;

}

using namespace TEASY_LASERS;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++Paramenter for Anchor Corrdinate System unit[m] relateive to utm origin+++++++++++++++++++++++++++++++++

namespace ACS
{
  //This is the origin of anchor coordinate system
  double anchor_x=0.0;
  double anchor_y=0.0;

  double history_anchor_x=0.0;
  double history_anchor_y=0.0;

  //This is border of car movement enviroment to make sure the car is alwasy on the center of the Gird

  double grid_border_x1=(GRID_SIZE_X*100/400)*GRID_SPACING;
  double grid_border_x2=(GRID_SIZE_X*300/400)*GRID_SPACING;
  double grid_border_y1=(GRID_SIZE_Y*100/400)*GRID_SPACING;
  double grid_border_y2=(GRID_SIZE_Y*300/400)*GRID_SPACING;
  

  double grid_trans_x=grid_border_x2-grid_border_x1;
  double grid_trans_y=grid_border_y2-grid_border_y1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define PUBLISH_DATA_




int main(int argc, char **argv)
{
  uint8_t current_grid[GRID_SIZE_X][GRID_SIZE_Y];
  for(int i=0;i<GRID_SIZE_X;i++)
    for(int j=0;j<GRID_SIZE_Y;j++)
     current_grid[i][j]=50;
  
  uint8_t history_grid[GRID_SIZE_X][GRID_SIZE_Y];
  for(int i=0;i<GRID_SIZE_X;i++)
    for(int j=0;j<GRID_SIZE_Y;j++)
     history_grid[i][j]=50;

  ros::init(argc,argv,"data_process");
  ros::NodeHandle nh;
  ros::Rate loop_rate(25);
  

  uint8_t (*current_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&current_grid;
  uint8_t (*history_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&history_grid;
  uint8_t (*testt_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&test_grid;

  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh,"/VehiclePoseFusionUTM",1000);
  message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZL>> pcl_sub(nh,"/as_tx/point_cloud",1000);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, pcl::PointCloud<pcl::PointXYZL>> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose_sub,pcl_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2,current_grid_pointer));

  ros::Publisher car_pub = nh.advertise<nav_msgs::GridCells>("car_visual",1000);
  ros::Publisher anchor_pub = nh.advertise<nav_msgs::GridCells>("acs_visual",1000);
  ros::Publisher pcl_pub = nh.advertise<nav_msgs::GridCells>("pcl_visual",1000);
  ros::Publisher unknow_area_pub = nh.advertise<nav_msgs::GridCells>("unknow_area",1000);
  ros::Publisher free_space_pub = nh.advertise<nav_msgs::GridCells>("free_space",1000);
  ros::Publisher test_pub = nh.advertise<nav_msgs::GridCells>("test_visual",1000);

  #ifdef PUBLISH_DATA_
  ros::Publisher pub=nh.advertise<grid_based_mapping_tao::to_visual>("data2visual",1000);
   
  grid_based_mapping_tao::to_visual data_to_pub;
  //++++++++++++++++setting the multiarray parameters These are constants++++++++++++++++++++++++++
  data_to_pub.array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  data_to_pub.array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  data_to_pub.array.layout.dim[0].label="grid_x";
  data_to_pub.array.layout.dim[1].label="grid_y";
  data_to_pub.array.layout.dim[0].size=GRID_SIZE_X;
  data_to_pub.array.layout.dim[1].size=GRID_SIZE_Y;
  data_to_pub.array.layout.dim[0].stride=GRID_SIZE_X*GRID_SIZE_Y;
  data_to_pub.array.layout.dim[1].stride=GRID_SIZE_Y;
  data_to_pub.array.layout.data_offset=0;
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  #endif



  //Wait a valid msg and then initialize the anchor position
  while(ros::ok())
  {
      ROS_INFO("(x,y,yaw)=( %f, %f, %f )",car.get_x(),car.get_y(),car.get_psi());
      if (car.get_x()!=0.0)
      {
        break;
      }    
    ros::spinOnce();
    loop_rate.sleep();
  }
  anchor_init(car);
  ROS_INFO("Anchor init_postion ( %f, %f )",ACS::anchor_x,ACS::anchor_y);

     
  //core func
  while(ros::ok())
  {
    #ifdef PUBLISH_DATA_
    data_to_pub.anchor.x=ACS::anchor_x;
    data_to_pub.anchor.y=ACS::anchor_y;

    data_to_pub.pose.x=car.get_x();
    data_to_pub.pose.y=car.get_y();
    data_to_pub.pose.theta=car.get_psi();
    #endif

    ros::spinOnce();

    geometry_msgs::Point cluster_point; //One point to be used as argument for all vectors push back

    //anchor++++
    std::vector<geometry_msgs::Point> anchor_cluster;
    cluster_point.x=ACS::anchor_x;
    cluster_point.y=ACS::anchor_y;
    cluster_point.z=0.0;   
    anchor_cluster.push_back(cluster_point);
        
    cluster_point.x=cluster_point.x+GRID_SIZE_X*GRID_SPACING;
    cluster_point.y=cluster_point.y+GRID_SIZE_X*GRID_SPACING;
    cluster_point.z=0.0; 
    anchor_cluster.push_back(cluster_point);
      
    //car++++
    std::vector<geometry_msgs::Point> car_grid;
    std::vector<geometry_msgs::Point> car_cluster;

    

    generate_car_cluster(car.get_x(),car.get_y(),car.get_psi(),ACS::anchor_x,ACS::anchor_y,car_grid);
     
    for(int i=0;i<car_grid.size();i++)
    {
       current_grid[uint16_t(car_grid[i].x)][uint16_t(car_grid[i].y)]=10;
       test_grid[uint16_t(car_grid[i].x)][uint16_t(car_grid[i].y)]=Grid_car;
    }
    
    for(int i=0;i<car_grid.size();i++)
    {
      cluster_point.x=car_grid[i].x*GRID_SPACING+ACS::anchor_x;
      cluster_point.y=car_grid[i].y*GRID_SPACING+ACS::anchor_y;
      cluster_point.z=0.0; 
      car_cluster.push_back(cluster_point);
    }

    //freespace(current_grid_pointer);

    adj(testt_grid_pointer);
    freespace_debug(current_grid_pointer,history_grid_pointer);

    #ifdef PUBLISH_DATA_
    data_to_pub.car_points=car_cluster;
    #endif
      
      
    Bayes(current_grid_pointer,history_grid_pointer);
    
    #ifdef PUBLISH_DATA_
    //++++++++++++++change the g_grid to a vector so it can change in message+++++
    //++++++++++++++++++++according std_msgs/UInt8MultiArray array++++++++++++++++
    std::vector<uint8_t> vec(GRID_SIZE_X*GRID_SIZE_Y,0);
    for(int i=0;i<GRID_SIZE_X;i++)
      for(int j=0;j<GRID_SIZE_Y;j++)
      { 
        vec[i*GRID_SIZE_Y+j]=history_grid[i][j];
      }
    data_to_pub.array.data=vec;
    #endif
      

      
    std::vector<geometry_msgs::Point> pcl_cluster_temp;
    std::vector<geometry_msgs::Point>  unknow_area;
    std::vector<geometry_msgs::Point>  freespace_area;

    
    for(int i=0;i<GRID_SIZE_X;i++)
      for(int j=0;j<GRID_SIZE_Y;j++)
	      {    
          
            if(history_grid[i][j]>binary_threshold)//grid[i][j]=g_data.array.data[i*GRID_SIZE_Y+j];
            {
               
              cluster_point.x=ACS::anchor_x+i*GRID_SPACING;
              cluster_point.y=ACS::anchor_y+j*GRID_SPACING;
              cluster_point.z=0.0;
              pcl_cluster_temp.push_back(cluster_point);
            }
            
            else if((history_grid[i][j]<binary_threshold)&&(test_grid[i][j]!=Grid_car))
            {
              cluster_point.x=ACS::anchor_x+i*GRID_SPACING;
              cluster_point.y=ACS::anchor_y+j*GRID_SPACING;
              cluster_point.z=0.0;
              freespace_area.push_back(cluster_point);
            }

            if(history_grid[i][j]==binary_threshold)
            {
              cluster_point.x=ACS::anchor_x+i*GRID_SPACING;
              cluster_point.y=ACS::anchor_y+j*GRID_SPACING;
              cluster_point.z=0.0;
              unknow_area.push_back(cluster_point);
            }
          
           
        } 

      
      visualize_girdcells(anchor_cluster,"map",float(1),float(1),anchor_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
      visualize_girdcells(pcl_cluster_temp,"map",float(GRID_SPACING),float(GRID_SPACING),pcl_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
      visualize_girdcells(freespace_area,"map",float(GRID_SPACING),float(GRID_SPACING),free_space_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
      visualize_girdcells(unknow_area,"map",float(GRID_SPACING),float(GRID_SPACING),unknow_area_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
      visualize_girdcells(car_cluster,"map",float(GRID_SPACING),float(GRID_SPACING),car_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
      visualize_girdcells(test_cluster,"map",float(GRID_SPACING),float(GRID_SPACING),test_pub,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);

     

      for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
            test_grid[i][j]=0;

      ACS::history_anchor_x=ACS::anchor_x;
      ACS::history_anchor_y=ACS::anchor_y;
      test_cluster.clear();

      #ifdef PUBLISH_DATA_
      pub.publish(data_to_pub);
      #endif
      

    
 
    
    loop_rate.sleep();
  }
  return 0;
}

  
  
 void callback(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg,const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& pcl_msg,uint8_t(*current_grid)[GRID_SIZE_X][GRID_SIZE_Y])
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
      
        //ROS_INFO("TIME STAMP FROM MSG POSE %llu.%6llu",pose_ts_s,pose_ts_ns);
        //ROS_INFO("TIME STAMP FROM MSG PCL2 %llu.%6llu",pcl_ts_s,pcl_ts_ns);
        //ROS_INFO("---------------------------------------------");

        bool skip_the_frame {false};//Flag to check if we should skip the frame
        //+++++++++++++++++PCL2 PROCESS++++++++++++++++++++++
        //Check if this frame has information about layers 4-7
        bool if_larger {false};
        BOOST_FOREACH(const pcl::PointXYZL& pt, pcl_msg->points)
        {
          if(pt.label>=5)
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
          adjust_ACS();

          for(int i=0;i<GRID_SIZE_X;i++)
          for(int j=0;j<GRID_SIZE_Y;j++)
            {
              (*current_grid)[i][j]=50;
            } 
          
            BOOST_FOREACH (const pcl::PointXYZL& pt, pcl_msg->points)
          {
            if((pt.label!=2)&&(pt.label!=3)) //We acess only layer 2-3
              {continue;} 

            //ROS_INFO("z is %f",pt.z);
            if((pt.z<1.8)&&((pt.z-0.3)>0))
            {
              geometry_msgs::Point pcl_acs;
              double pos_psy=(-car.get_psi()+90)/180*M_PI;
              pcl_acs.x=cos(pos_psy)*pt.x-sin(pos_psy)*pt.y;
              pcl_acs.y=sin(pos_psy)*pt.x+cos(pos_psy)*pt.y;
              pcl_acs.x=pcl_acs.x+car.get_x()-ACS::anchor_x;
              pcl_acs.y=pcl_acs.y+car.get_y()-ACS::anchor_y;
              if( (pcl_acs.x>=0) && (pcl_acs.x<=double(GRID_SIZE_X*GRID_SPACING)) && (pcl_acs.y>=0) && (pcl_acs.y<=double(GRID_SIZE_Y*GRID_SPACING)) )
              {
                uint16_t grid_x=uint16_t(floor(pcl_acs.x/GRID_SPACING));
                uint16_t grid_y=uint16_t(floor(pcl_acs.y/GRID_SPACING));
                (*current_grid)[grid_x][grid_y]=90;
              } 
            }
            
          }
          
          
        }

        

       
      }

    
     
}


void anchor_init(const Vehicle& car)
{
  //Make the anchor near the car so that it could quickly determine its initilazied position and make sure it is 10*Z number
  ACS::anchor_x= double(static_cast<int64_t>(car.get_x())/10)*10.0;
  ACS::anchor_y= double(static_cast<int64_t>(car.get_y())/10)*10.0;

  

  while((car.get_x()<ACS::anchor_x+ACS::grid_border_x1)||(car.get_x()>ACS::anchor_x+ACS::grid_border_x2)||(car.get_y()<ACS::anchor_y+ACS::grid_border_y1)||(car.get_x()>ACS::anchor_y+ACS::grid_border_y2))
  {
    adjust_ACS();
  }

  ACS::history_anchor_x=ACS::anchor_x;
  ACS::history_anchor_y=ACS::anchor_y;

}


void adjust_ACS()
{
  /* 
  if( (car.get_psi()>=22.5) && (car.get_psi()<=67.5) )
    {
      ACS::grid_border_x1=(GRID_SIZE_X*1/5)*GRID_SPACING;
      ACS::grid_border_x2=(GRID_SIZE_X*2/5)*GRID_SPACING;
      ACS::grid_border_y1=(GRID_SIZE_Y*1/5)*GRID_SPACING;
      ACS::grid_border_y2=(GRID_SIZE_Y*2/5)*GRID_SPACING;
    }
    else if( (car.get_psi()>=67.5) && (car.get_psi()<=112.5) )
    {
      ACS::grid_border_x1=(GRID_SIZE_X*1/5)*GRID_SPACING;
      ACS::grid_border_x2=(GRID_SIZE_X*2/5)*GRID_SPACING;;
      ACS::grid_border_y1=(GRID_SIZE_Y*2/5)*GRID_SPACING;
      ACS::grid_border_y2=55;
    }
    else if( (car.get_psi()>=112.5) && (car.get_psi()<=157.5) )
    {
      ACS::grid_border_x1=20;
      ACS::grid_border_x2=30;
      ACS::grid_border_y1=70;
      ACS::grid_border_y2=80;
    }
    else if( (car.get_psi()>=157.5) && (car.get_psi()<=202.5) )
    {
      ACS::grid_border_x1=45;
      ACS::grid_border_x2=55;
      ACS::grid_border_y1=70;
      ACS::grid_border_y2=80;
    }
    else if( (car.get_psi()>=202.5) && (car.get_psi()<=247.5) )
    {
      ACS::grid_border_x1=70;
      ACS::grid_border_x2=80;
      ACS::grid_border_y1=70;
      ACS::grid_border_y2=80;
    }
    else if( (car.get_psi()>=247.5) && (car.get_psi()<=292.5) )
    {
      ACS::grid_border_x1=70;
      ACS::grid_border_x2=80;
      ACS::grid_border_y1=45;
      ACS::grid_border_y2=55;
    }
    else if( (car.get_psi()>=292.5) && (car.get_psi()<=337.5) )
    {
      ACS::grid_border_x1=70;
      ACS::grid_border_x2=80;
      ACS::grid_border_y1=20;
      ACS::grid_border_y2=30;
    }
    else
  {
      ACS::grid_border_x1=45;
      ACS::grid_border_x2=55;
      ACS::grid_border_y1=20;
      ACS::grid_border_y2=30;
  }

  ACS::grid_trans_x = ACS::grid_border_x2 - ACS::grid_border_x1;
  ACS::grid_trans_y = ACS::grid_border_y2 - ACS::grid_border_y1;
*/

  //adjust the anchor of ACS if it is necessary
  if (car.get_x()<(ACS::anchor_x+ACS::grid_border_x1))
  {
    ACS::anchor_x=ACS::anchor_x-ACS::grid_trans_x;
  }
  else if (car.get_x()>(ACS::anchor_x+ACS::grid_border_x2))
  {
    ACS::anchor_x=ACS::anchor_x+ACS::grid_trans_x;
  }
  if(car.get_y()<(ACS::anchor_y+ACS::grid_border_y1))
  {
    ACS::anchor_y=ACS::anchor_y-ACS::grid_trans_y;
  }
  else if(car.get_y()>(ACS::anchor_y+ACS::grid_border_y2))
  {
    ACS::anchor_y=ACS::anchor_y+ACS::grid_trans_y;
  }

}


//Generate the discretized car cluster relative to ACS 
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
    cluster_x_field.push_back(uint32_t(floor(cluster_x[i]/GRID_SPACING)));
  }
  for(int i=0; i<cluster_y.size();i++)
  {
    cluster_y_field.push_back(uint32_t(floor(cluster_y[i]/GRID_SPACING)));
  }

  geometry_msgs::Point single_point;

  for(int i=0; i<cluster_x_field.size();i++)
  {
    
    single_point.x=cluster_x_field[i];
    single_point.y=cluster_y_field[i];
    car_grid.push_back(single_point);
  }

}


std::vector<double> linspace(double a,double b, int n)
{
  std::vector<double> array;
  double step=(b-a)/(n-1);
  while(a<=b)
    {
      array.push_back(a);
      a+=step;
    }
  return array;
}


void freespace(uint8_t (*array)[GRID_SIZE_X][GRID_SIZE_Y])
{
  
  double offset_x=car.get_x()-ACS::anchor_x;
  double offset_y=car.get_y()-ACS::anchor_y;

  for(int laser_num=0;laser_num<laser_x.size();laser_num++)
  {

    double pos_psy=(-car.get_psi()+90)/180*M_PI;
    double laser_abs_x=cos(pos_psy)*laser_x[laser_num]-sin(pos_psy)*laser_y[laser_num]+offset_x;
    double laser_abs_y=sin(pos_psy)*laser_x[laser_num]+cos(pos_psy)*laser_y[laser_num]+offset_y;

    double laser_abs_yaw=std::fmod(pos_psy+laser_yaw[laser_num],2*M_PI);

    int32_t xstart=floor(laser_abs_x/GRID_SPACING);
    int32_t ystart=floor(laser_abs_y/GRID_SPACING);

     /*  
    geometry_msgs::Point test_point;
    test_point.x=xstart*GRID_SPACING+ACS::anchor_x;
    test_point.y=ystart*GRID_SPACING+ACS::anchor_y;
    test_point.z=0.0;
    test_cluster.push_back(test_point);
    */
    //ROS_INFO("xstart %d",xstart);
    //ROS_INFO("ystart %d",ystart);
  
    
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
      
    /* 
      geometry_msgs::Point test_point;
      test_point.x=xend*GRID_SPACING+ACS::anchor_x;
      test_point.y=yend*GRID_SPACING+ACS::anchor_y;
      test_point.z=0.0;
      test_cluster.push_back(test_point);
      */
      //ROS_INFO("xend %d",xend);
      //ROS_INFO("yend %d",yend);


      int32_t dx=round(xend-xstart);
      int32_t dy=round(yend-ystart);

      //ROS_INFO("dx %d",dx);
      //ROS_INFO("dy %d",dy);

      int incx=sign(dx);
      int incy=sign(dy);
      
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

      for(int32_t j=0;j<(deltafastdirection-1);j++)
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
       
        if((*array)[uint16_t(x)][uint16_t(y)]==90)
          {break;}
        else if((*array)[uint16_t(x)][uint16_t(y)]!=50)
          {continue;}
        else if((*array)[uint16_t(x)][uint16_t(y)]==50)
        {
          double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
          if (d<DMIN)
          {
            (*array)[uint16_t(x)][uint16_t(y)]=20;
          
            }
          else if((d<DMAX)&&(d>=DMIN))
          {
            (*array)[uint16_t(x)][uint16_t(y)]=20+0.5*(d-DMIN);
          }   
        }
           
      }
    
    }
  }

}




void freespace_debug(uint8_t (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y])
{
  
  double offset_x=car.get_x()-ACS::anchor_x;
  double offset_y=car.get_y()-ACS::anchor_y;

  for(int laser_num=0;laser_num<laser_x.size();laser_num++)
  {

    double pos_psy=(-car.get_psi()+90)/180*M_PI;
    double laser_abs_x=cos(pos_psy)*laser_x[laser_num]-sin(pos_psy)*laser_y[laser_num]+offset_x;
    double laser_abs_y=sin(pos_psy)*laser_x[laser_num]+cos(pos_psy)*laser_y[laser_num]+offset_y;

    double laser_abs_yaw=std::fmod(pos_psy+laser_yaw[laser_num],2*M_PI);

    int32_t xstart=floor(laser_abs_x/GRID_SPACING);
    int32_t ystart=floor(laser_abs_y/GRID_SPACING);

    /* 
    geometry_msgs::Point test_point;
    test_point.x=xstart*GRID_SPACING+ACS::anchor_x;
    test_point.y=ystart*GRID_SPACING+ACS::anchor_y;
    test_point.z=0.0;
    test_cluster.push_back(test_point);
    */
    //ROS_INFO("xstart %d",xstart);
    //ROS_INFO("ystart %d",ystart);
  
    
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
      
      /* 
      geometry_msgs::Point test_point;
      test_point.x=xend*GRID_SPACING+ACS::anchor_x;
      test_point.y=yend*GRID_SPACING+ACS::anchor_y;
      test_point.z=0.0;
      test_cluster.push_back(test_point);
      */
      //ROS_INFO("xend %d",xend);
      //ROS_INFO("yend %d",yend);


      int32_t dx=round(xend-xstart);
      int32_t dy=round(yend-ystart);

      //ROS_INFO("dx %d",dx);
      //ROS_INFO("dy %d",dy);

      int incx=sign(dx);
      int incy=sign(dy);
      
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

      for(int32_t j=0;j<(deltafastdirection-1);j++)
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
       
        if((*history_grid)[uint16_t(x)][uint16_t(y)]>binary_threshold)
          {
            test_grid[uint16_t(x)][uint16_t(y)]=Grid_occupied;


            break;
          }
        else if((*current_grid)[uint16_t(x)][uint16_t(y)]!=50)
          {continue;}
        else if((*current_grid)[uint16_t(x)][uint16_t(y)]==50)
        {
          double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
          if (d<DMIN)
          {
            (*current_grid)[uint16_t(x)][uint16_t(y)]=20;
          
            }
          else if((d<DMAX)&&(d>=DMIN))
          {
            (*current_grid)[uint16_t(x)][uint16_t(y)]=20+0.5*(d-DMIN);
          }   
        }
           
      }
    
    }
  }

}

void adj(uint8_t (*grid)[GRID_SIZE_X][GRID_SIZE_Y])
{
  uint16_t grid_trans_y=std::abs((ACS::anchor_y-ACS::history_anchor_y)/GRID_SPACING);
  uint16_t grid_trans_x=std::abs((ACS::anchor_x-ACS::history_anchor_x)/GRID_SPACING);

  if(ACS::anchor_x<ACS::history_anchor_x)
  {
    for(int i=grid_trans_x;i<GRID_SIZE_X;i++)
      for(int j=0;j<GRID_SIZE_Y;j++)
      { 
        (*grid)[i][j]=(*grid)[i-grid_trans_x][j];
      }

      for(int i=0;i<grid_trans_x;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
        { 
          (*grid)[i][j]=50;
        }
          

  }
      
  else if(ACS::anchor_x>ACS::history_anchor_x)
    {
      for(int i=0;i<GRID_SIZE_X-grid_trans_x;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
        { 
          (*grid)[i][j]=(*grid)[i+grid_trans_x][j];
        }

      for(int i=GRID_SIZE_X-grid_trans_x;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
        { 
          (*grid)[i][j]=50;
        }  

    }
  
  if(ACS::anchor_y<ACS::history_anchor_y)
    {
      for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=grid_trans_y;j<GRID_SIZE_Y;j++)
        { 
         (*grid)[i][j]=(*grid)[i][j-grid_trans_y];
        }

      for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<grid_trans_y;j++)
        { 
          (*grid)[i][j]=50;
        }

    }
      
  else if(ACS::anchor_y>ACS::history_anchor_y)
    {
      
        for(int i=0;i<GRID_SIZE_X;i++)
          for(int j=0;j<GRID_SIZE_Y-grid_trans_y;j++)
          { 
            (*grid)[i][j]=(*grid)[i][j+grid_trans_y];
          }
          
        
        for(int i=0;i<GRID_SIZE_X;i++)
          for(int j=GRID_SIZE_Y-grid_trans_y;j<GRID_SIZE_Y;j++)
          { 
            (*grid)[i][j]=50;
          }  
            
    }
    
}
void Bayes(uint8_t (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y])
{
  
  uint16_t grid_trans_y=std::abs((ACS::anchor_y-ACS::history_anchor_y)/GRID_SPACING);
  uint16_t grid_trans_x=std::abs((ACS::anchor_x-ACS::history_anchor_x)/GRID_SPACING);



  if(ACS::anchor_x<ACS::history_anchor_x)
  {
    for(int i=grid_trans_x;i<GRID_SIZE_X;i++)
      for(int j=0;j<GRID_SIZE_Y;j++)
      { 
        (*history_grid)[i][j]=(*history_grid)[i-grid_trans_x][j];
      }

      for(int i=0;i<grid_trans_x;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
        { 
          (*history_grid)[i][j]=50;
        }
          

  }
      
  else if(ACS::anchor_x>ACS::history_anchor_x)
    {
      for(int i=0;i<GRID_SIZE_X-grid_trans_x;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
        { 
          (*history_grid)[i][j]=(*history_grid)[i+grid_trans_x][j];
        }

      for(int i=GRID_SIZE_X-grid_trans_x;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
        { 
          (*history_grid)[i][j]=50;
        }  

    }
  
  if(ACS::anchor_y<ACS::history_anchor_y)
    {
      for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=grid_trans_y;j<GRID_SIZE_Y;j++)
        { 
         (*history_grid)[i][j]=(*history_grid)[i][j-grid_trans_y];
        }

      for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<grid_trans_y;j++)
        { 
          (*history_grid)[i][j]=50;
        }

    }
      
  else if(ACS::anchor_y>ACS::history_anchor_y)
    {
      
        for(int i=0;i<GRID_SIZE_X;i++)
          for(int j=0;j<GRID_SIZE_Y-grid_trans_y;j++)
          { 
            (*history_grid)[i][j]=(*history_grid)[i][j+grid_trans_y];
          }
          
        
        for(int i=0;i<GRID_SIZE_X;i++)
          for(int j=GRID_SIZE_Y-grid_trans_y;j<GRID_SIZE_Y;j++)
          { 
            (*history_grid)[i][j]=50;
          }  
            
    }

  

  for(int i=0;i<GRID_SIZE_X;i++)
    for(int k=0;k<GRID_SIZE_Y;k++)
        {
          
          if (((*current_grid)[i][k])!=50)
          {
            double temp=double((*current_grid)[i][k]);
            double p_cur=temp;
            temp=double((*history_grid)[i][k]);
            double p_hist=temp;
            double s=p_cur/(100-p_cur)*p_hist/(100-p_hist);
            (*history_grid)[i][k]=uint8_t(100.0*s/(s+1));

          }
          else
          {
            (*history_grid)[i][k]=(*history_grid)[i][k];
          }

          if ((*history_grid)[i][k]>(100-epsilon))
          {
            (*history_grid)[i][k]=100-epsilon;
          }
          else if((*history_grid)[i][k]<(epsilon))
          {
            (*history_grid)[i][k]=epsilon;
          }

        }

}





double sind(double value_in_degree)
{
  double  value_in_radius= value_in_degree*M_PI/180;
  return sin(value_in_radius);
}


double cosd(double value_in_degree)
{
  double  value_in_radius= value_in_degree*M_PI/180;
  return cos(value_in_radius);
}


int sign(int x)
{
  if (x>0) return 1;
  if (x<0) return -1;
  return 0;
}




//visualize Cells
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


template <size_t row, size_t col>
void print_out_grid(uint8_t (&array)[row][col])
{
   std_msgs::String print_msg;
   std::stringstream ss_grid;
   ss_grid<<"\n";

    for(int i=0;i<row;i++)
    {
      for(int j=0;j<col;j++)
      {
        ss_grid<<std::to_string(array[i][j])<<" ";
      }
      ss_grid<<"\n";
    }
    print_msg.data=ss_grid.str();
    ROS_INFO("%s",print_msg.data.c_str());
}