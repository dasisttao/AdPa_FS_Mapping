#include "grid_based_mapping_tao/parameter_grid.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/OccupancyGrid.h>

/*++++++++++++++++++++++++++This node is doing the following  jobs+++++++++++++++++++++++++++++++++++++++++
    
        Get Message from Topic /VehiclePoseFusionUTM so that we can get the car pose 
        Get Message from Topic /as_tx/point_cloud so that we can get the pcl message

        Let pcl discretized in grid and use bayes filter to estimate the probabilty of the occupization


      Publish the Message to Topic /data2visual so that we can visualize it with rviz.The Message conatains
      car cluster, anchor position and occupancygrid(history_grid)(by a varibale of type std_msgs/UInt8MultiArray)

 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
float l_50= log(50.0/(100.0-50.0));
float l_20= log(20.0/(100.0-20.0));
float l_90= log(90.0/(100.0-90.0));
float l_99= log(99.0/(100.0-99.0));
float l_1=  log(1.0/(100.0-1.0));


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

 //++++++++++++++++++++++++++++++++++++++++++++++++CONFIG/PARAMETER++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint64_t allowed_diff_ts_ns=25000;//0.025s -the allowed difference of timestamp between msg of car_pose and msg of pcl2


namespace TEASY_LASERS
{
    double DMAX=10; //max distannce of reachement
    double DMIN=1;
    std::vector<double> laser_x {+3.6,-1.0,+3.3,+3.3,-0.5,-0.5}; //In VCS Vehicle Coordinate System
    std::vector<double> laser_y {+0.0,+0.0,-0.8,+0.8,-0.8,+0.8};
    std::vector<double> laser_yaw {-45*M_PI/180,-225*M_PI/180,-125*M_PI/180,35*M_PI/180,-145*M_PI/180,55*M_PI/180};

    //std::vector<double> laser_x {+3.6,+3.3}; //In VCS Vehicle Coordinate System
    //std::vector<double> laser_y {+0.0,-0.8};
    //std::vector<double> laser_yaw {-45*M_PI/180,-125*M_PI/180};

    double angle_resolution=GRID_SPACING/DMAX;
    uint32_t radiation_num=static_cast<uint32_t>(2*M_PI/angle_resolution);
    uint8_t radiation_num_divder=4;

}

using namespace TEASY_LASERS;


int sign(int x)
{
    if (x>0) return 1;
    else if (x<0) return -1;
    return 0;
}
 //++++++++++++++++++++++++++++++++++++++++++++++++global varibale+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Vehicle car;//create global object car 

bool if_get_new_data {false};

//++++++++++++++++++++++++++++Paramenter for Anchor Corrdinate System unit[m] relateive to utm origin+++++++++++++++++++++++++++++++++

namespace ACS
{
    //This is the origin of anchor coordinate system
    double anchor_x {0.0};
    double anchor_y {0.0};

    double history_anchor_x {0.0};
    double history_anchor_y {0.0};

    //This is border of car movement enviroment to make sure the car is alwasy on the center of the Gird
    double grid_border_x1=(GRID_SIZE_X*100/400)*GRID_SPACING;
    double grid_border_x2=(GRID_SIZE_X*300/400)*GRID_SPACING;
    double grid_border_y1=(GRID_SIZE_Y*100/400)*GRID_SPACING;
    double grid_border_y2=(GRID_SIZE_Y*300/400)*GRID_SPACING;
  

    double grid_trans_x=grid_border_x2-grid_border_x1;
    double grid_trans_y=grid_border_y2-grid_border_y1;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++Functions++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void update_grid_when_anchor_moved(float (*grid)[GRID_SIZE_X][GRID_SIZE_Y],float new_are_fullfill);
void callback(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg,const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& pcl_msg,uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y]);
void adjust_ACS();// according to the current pose of car,update the ACS anchor
void anchor_init(const Vehicle& car);// according to the initial pose of car, initiliaze the ACS anchor. 
void generate_car_cluster(double ego_x,double ego_y,double ego_yaw,double anchor_x,double ancor_y,std::vector<geometry_msgs::Point> &car_grid);
std::vector<double> linspace(double a,double b, int n);
void visualize_girdcells(const std::vector<geometry_msgs::Point>& cells,const std::string& frame_id,float grid_cell_width,float grid_cell_height,
                        ros::Publisher pub,double offset_x,double offset_y);


void ray_casting_points(uint8_t(*ray_casting)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t(*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t(*current_grid)[GRID_SIZE_X][GRID_SIZE_Y]);

void create_freespace(uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y]);

void binary_bayes(float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y]);
//++++++++++++++++++++++++++++++++++++++++++++++++++MAIN++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
    float current_grid[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
            current_grid[i][j]=l_50;

    float history_grid[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
            history_grid[i][j]=l_50;


    uint8_t pcl_grid[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int j=0;j<GRID_SIZE_Y;j++)
            pcl_grid[i][j]=50;
  
    ros::init(argc,argv,"log");
    ros::NodeHandle nh;
    ros::Rate loop_rate(25);

    float (*current_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&current_grid;
    float (*history_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&history_grid;
    uint8_t (*pcl_grid_pointer)[GRID_SIZE_X][GRID_SIZE_Y]=&pcl_grid;

    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh,"/VehiclePoseFusionUTM",1000);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZL>> pcl_sub(nh,"/as_tx/point_cloud",1000);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, pcl::PointCloud<pcl::PointXYZL>> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose_sub,pcl_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,pcl_grid_pointer));

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
    anchor_init(car);
    //ROS_INFO("Anchor init_postion ( %f, %f )",ACS::anchor_x,ACS::anchor_y);

    while(ros::ok())
    {
        ros::spinOnce();//call all the callbacks waiting to be called at that point in time.

        if(if_get_new_data==true)
        {
            geometry_msgs::Pose og_origin;
            og_origin.position.x=ACS::anchor_x-VISUAL_OFFSET_X;
            og_origin.position.y=ACS::anchor_y-VISUAL_OFFSET_Y;
            og_origin.position.z=0.0;  
            oG2D oG2D(0.1,GRID_SIZE_X,GRID_SIZE_Y,og_origin);



            geometry_msgs::Point cluster_point; //One point to be used as argument for all vectors push back

            //anchor++++
            std::vector<geometry_msgs::Point> anchor_cluster;
            cluster_point.x=ACS::anchor_x;
            cluster_point.y=ACS::anchor_y;
            cluster_point.z=0.0;   
            anchor_cluster.push_back(cluster_point);

            //car++++
            std::vector<geometry_msgs::Point> car_grid;
            generate_car_cluster(car.get_x(),car.get_y(),car.get_psi(),ACS::anchor_x,ACS::anchor_y,car_grid);
            
            for(int i=0;i<car_grid.size();i++)
            {
                current_grid[uint16_t(car_grid[i].x)][uint16_t(car_grid[i].y)]=l_20;
            }
        
            std::vector<geometry_msgs::Point> car_cluster;
            for(int i=0;i<car_grid.size();i++)
            {
                cluster_point.x=car_grid[i].x*GRID_SPACING+ACS::anchor_x;
                cluster_point.y=car_grid[i].y*GRID_SPACING+ACS::anchor_y;
                cluster_point.z=0.0; 
                car_cluster.push_back(cluster_point);
            }

        

            //ray_casting_points(ray_casting_pointer,pcl_grid_pointer,current_grid_pointer);
            create_freespace(pcl_grid_pointer,current_grid_pointer);
            update_grid_when_anchor_moved(history_grid_pointer,l_50);
            binary_bayes(current_grid_pointer,history_grid_pointer);

            std::vector<int8_t> probability_vector;

        
            for(auto j=0;j<GRID_SIZE_Y;j++)
                for(auto i=0;i<GRID_SIZE_X;i++)
                {
                    if(history_grid[i][j]==l_50)
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
                    current_grid[i][j]=l_50;
                }


            ACS::history_anchor_x=ACS::anchor_x;
            ACS::history_anchor_y=ACS::anchor_y;
            if_get_new_data=false;
        }


    }




}


  








//++++++++++++++++++++++++++++++++++++++++++++++++++Function definition++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void update_grid_when_anchor_moved(float (*grid)[GRID_SIZE_X][GRID_SIZE_Y],float new_are_fullfill)
{
    if((ACS::anchor_x==ACS::history_anchor_x)&&(ACS::anchor_y==ACS::history_anchor_y))
    {return;}

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
                (*grid)[i][j]=new_are_fullfill;
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
                (*grid)[i][j]=new_are_fullfill;
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
                (*grid)[i][j]=new_are_fullfill;
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
                (*grid)[i][j]=new_are_fullfill;
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


 void callback(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg,const pcl::PointCloud<pcl::PointXYZL>::ConstPtr& pcl_msg,uint8_t(*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y])
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
        
        car.set_x(pose_msg->pose.position.x);
        car.set_y(pose_msg->pose.position.y);
        car.set_psi(pose_msg->pose.position.z); 
        adjust_ACS();

          
        for(int i=0;i<GRID_SIZE_X;i++)
            for(int j=0;j<GRID_SIZE_Y;j++)
            {
                (*pcl_grid)[i][j]=50;
            } 
          
        BOOST_FOREACH (const pcl::PointXYZL& pt, pcl_msg->points)
        {
            if(pt.label==0)
            {
                continue;
            }
            if((pt.z<2.8)&&((pt.z-0.1)>0))
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
                    (*pcl_grid)[grid_x][grid_y]=90;
                } 
            }
            
        }
          
          if_get_new_data=true;
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



void ray_casting_points(uint8_t(*ray_casting)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t(*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],uint8_t(*current_grid)[GRID_SIZE_X][GRID_SIZE_Y])
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


      bool if_obstacle {false};
      bool if_one_obs {false};
      bool free_behind_wall {false};
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
        

        
        if((*pcl_grid)[uint16_t(x)][uint16_t(y)]==90)
        {
         
          if((d<DMAX)&&(d>=DMIN))
          {
            (*current_grid)[uint16_t(x)][uint16_t(y)]=90;
          }
        }
        
      
      }
       
    
    }
  }
}







void create_freespace(uint8_t (*pcl_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y])
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
                
                if((*pcl_grid)[uint16_t(x)][uint16_t(y)]==90)
                {
                
                    if((d<DMAX)&&(d>=DMIN))
                    {
                        (*current_grid)[uint16_t(x)][uint16_t(y)]=l_90;
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

                    if((*current_grid)[uint16_t(x)][uint16_t(y)]==l_90)
                    {
                        break;
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]==l_50)
                    {
                        double d=sqrt( pow((x-xstart)*GRID_SPACING,2.0)+ pow((y-ystart)*GRID_SPACING,2.0) );
                        if (d<DMIN)
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l_20;
                        }
                        else if((d<DMAX)&&(d>=DMIN))
                        {
                            (*current_grid)[uint16_t(x)][uint16_t(y)]=l_20;
                            //(*current_grid)[uint16_t(x)][uint16_t(y)]=20+0.5*(d-DMIN);
                        }
                    }
                    else if((*current_grid)[uint16_t(x)][uint16_t(y)]!=l_50)
                    {continue;}
                    
                
                }
            }
            
        }
    }
}



void binary_bayes(float (*current_grid)[GRID_SIZE_X][GRID_SIZE_Y],float (*history_grid)[GRID_SIZE_X][GRID_SIZE_Y])
{
    for(int i=0;i<GRID_SIZE_X;i++)
        for(int k=0;k<GRID_SIZE_Y;k++)
            {
                if (((*current_grid)[i][k])!=l_50)
                {
                    (*history_grid)[i][k]=(*current_grid)[i][k]+(*history_grid)[i][k];
                }
                else 
                {
                    (*history_grid)[i][k]=(*history_grid)[i][k];
                }
                
                if((*history_grid)[i][k]>l_99)
                {
                    (*history_grid)[i][k]=l_99;
                }
                else if ((*history_grid)[i][k]<l_1)
                {
                    (*history_grid)[i][k]=l_1;
                }
            }
            
}