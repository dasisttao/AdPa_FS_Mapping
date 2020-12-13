#include "grid_based_mapping_tao/parameter_grid.h"
#include "nav_msgs/Path.h"

/*++++++++++++++++++++++++++This node is doing the following  jobs+++++++++++++++++++++++++++++++++++++++++
    
        Get Message from Topic /data2visual so that we can visualize the car and grid.

      


 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



//++++++++++++++++++++++++++++++++++++++++++++++++++Functions++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void callback_sub(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg);

std::vector<double> linspace(double a,double b, int n);
std::vector<geometry_msgs::Point> generate_car_cluster(double ego_x,double ego_y,double ego_yaw,double offset2world_x,double offset2world_y);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++global varibale+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
nav_msgs::Path vis_path;

nav_msgs::GridCells car_cluster;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++Flag++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool get_new_data {false};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}



int main(int argc, char **argv)
{
  

  ros::init(argc,argv,"path_visualization");
  ros::NodeHandle n;
  
  ros::Rate loop_rate(100);
  
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_visual",1000);

  ros::Publisher car_pub = n.advertise<nav_msgs::GridCells>("car_visual",1000);

  ros::Subscriber sub=n.subscribe<geometry_msgs::PoseStamped>("/VehiclePoseFusionUTM",1,callback_sub);

  vis_path.header.frame_id="/map";
  car_cluster.header.frame_id="/map";
  car_cluster.cell_width=GRID_SPACING;
  car_cluster.cell_height=GRID_SIZE_Y;
 
  while(ros::ok())
  {
    
    if(get_new_data==true)
    {
      vis_path.header.stamp=ros::Time::now();
      

      car_cluster.header.stamp=ros::Time::now();
      

      if(vis_path.poses.size()<100)
      {
        ROS_INFO("Pose number: %d",vis_path.poses.size());
      }
      if(vis_path.poses.size()>=100)
      {
        ROS_INFO("Print out the paht!");
        path_pub.publish(vis_path);
        car_pub.publish(car_cluster);
        while(ros::ok());
      }


    

      get_new_data=false;
    }
    

    
    ros::spinOnce();
    loop_rate.sleep();
    
  }


}



void callback_sub(const  geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  if(get_new_data==false)
  { 
    static uint32_t divider_count {0};
    divider_count++;
    if(divider_count>20)
    {
      divider_count=0;
     
      double yaw=(-pose_msg->pose.position.z+90)*M_PI/180;
      Quaternion q=ToQuaternion(yaw,0,0);

      geometry_msgs::PoseStamped single_pose;
      single_pose.pose.position.x=pose_msg->pose.position.x-VISUAL_OFFSET_X;
      single_pose.pose.position.y=pose_msg->pose.position.y-VISUAL_OFFSET_Y;
      single_pose.pose.position.z=0.0;

      single_pose.pose.orientation.w=q.w;
      single_pose.pose.orientation.y=q.y;
      single_pose.pose.orientation.x=q.x;
      single_pose.pose.orientation.z=q.z;

      vis_path.poses.push_back(single_pose);


      std::vector<geometry_msgs::Point> car_cluster_single=generate_car_cluster(single_pose.pose.position.x,single_pose.pose.position.y,pose_msg->pose.position.z,VISUAL_OFFSET_X,VISUAL_OFFSET_Y);
      car_cluster.cells.insert(car_cluster.cells.end(),car_cluster_single.begin(),car_cluster_single.end());
      
    }
    

    get_new_data=true;
  }
  
  
}


//Generate the car cluster and also move offset_x offset_y so it's good to viusalize
std::vector<geometry_msgs::Point> generate_car_cluster(double ego_x,double ego_y,double ego_yaw,double offset2world_x,double offset2world_y)
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

  for(int i=0; i<cluster_x_rot.size();i++)
  {
    cluster_x.push_back(cluster_x_rot[i]+ego_x);
  }
  for(int i=0; i<cluster_y_rot.size();i++)
  {
    cluster_y.push_back(cluster_y_rot[i]+ego_y);
  }



  std::vector<geometry_msgs::Point> vector_return;
  geometry_msgs::Point single_point;
  for(int i=0; i<cluster_x.size();i++)
  {
    single_point.x=cluster_x[i]-offset2world_x;
    single_point.y=cluster_y[i]-offset2world_y;
    vector_return.push_back(single_point);
  }

  return vector_return;

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
