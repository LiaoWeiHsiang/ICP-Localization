#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include <sstream>
#include <pcl/registration/icp.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "sensor_msgs/Imu.h"
#include "opencv2/core/core.hpp"
#include <math.h>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string> 
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <iostream>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <fstream>
#include <tf_conversions/tf_eigen.h>
#include <ctime>
#include <tf/transform_broadcaster.h>

using namespace Eigen;
using namespace std;

class SubscribeAndPublish  
{  

    
    public:  
      SubscribeAndPublish()  
      {  
        
        sub_raw_imu = nh.subscribe("/imu/data", 0.3, &SubscribeAndPublish::imu_callback, this);
        sub_lidar = nh.subscribe("/lidar_points", 10000, &SubscribeAndPublish::lidar_callback, this);
        sub_map = nh.subscribe("/map", 10000, &SubscribeAndPublish::map_callback, this);
        sub_gps = nh.subscribe("/fix", 10000, &SubscribeAndPublish::gps_callback, this);
        pub_lidar_icped = nh.advertise<sensor_msgs::PointCloud2> ("lidar_after_icp", 1);
        sub_tf = nh.subscribe("/tf", 10000, &SubscribeAndPublish::tf_callback, this);
        pub_position = nh.advertise<visualization_msgs::Marker>( "position", 0 );
      }  

    private:  
      ros::NodeHandle nh;    
      ros::Subscriber sub_raw_imu;  
      ros::Subscriber sub_gps; 
      ros::Subscriber sub_lidar;
      ros::Subscriber sub_map;
      ros::Publisher pub_lidar_icped;
      ros::Subscriber sub_tf;
      ros::Publisher pub_position;

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg3); 
    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg); 
    void gps_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void map_callback(const sensor_msgs::PointCloud2ConstPtr& map_msg); 
    



};

int gps_time=1;
int fre = 1;
float last_time;
double config_position_z;
Eigen::Matrix4d tf_transformation;
int theta;
double position_x;
double position_y;
double position_z;
Eigen::Vector3d imu_eulerAngle;
Eigen::Matrix4d transformation;
Eigen::Matrix4f initial_guess;

pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
geometry_msgs::PointStamped gps_point;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/map", 1, true);
  pcl::PointCloud<pcl::PointXYZI>::Ptr total_pcd_ptr (new pcl::PointCloud<pcl::PointXYZI>);
  string map_file = "/data/david/david_ws/src/sdc_localization/ITRI_data/map.pcd";
  pcl::io::loadPCDFile<pcl::PointXYZI> (map_file, *total_pcd_ptr);
  sensor_msgs::PointCloud2 cloudc_ros;
  pcl::toROSMsg (*total_pcd_ptr, cloudc_ros);
  cloudc_ros.header.stamp = ros::Time::now();
  cloudc_ros.header.frame_id = "map";
  pub_map.publish(cloudc_ros);
  SubscribeAndPublish test;
  ros::MultiThreadedSpinner s(2);  
  ros::spin(s);  
  return 0;   
}

int getdir( string dir,  vector<string> &files){
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dir.c_str())) == NULL){
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }
    while((dirp = readdir(dp)) != NULL){
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}



void SubscribeAndPublish::lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
cout << "----------------- = " << endl;
cout << "fre = " << fre << endl;
cout << "ros_time = " << msg->header.stamp << endl;
pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::fromROSMsg (*msg, *cloud);//cloud is the output 
pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
float smallest_score;
float smallest_tras;
double score;

// ============== First scan use GPS as initial guess and find a good matching ===========
if(fre==1){
  smallest_score = 1000;
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  int smallest_theta;
  //  === GPS provide a (x,y,z) coordindate, so we use a "for loop" to find a good "rotation initial guess" ===
  // itri_public theta = 234
  // itri_private_1 204
  // itri_private_2 138
  // itri_private_3 138

  for(int theta=234; theta<=234; theta+=2){ 
    double PI_angle = theta * M_PI/180;
    cout << "theta = " << theta << endl;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << position_x, position_y, position_z;
    transform.rotate (Eigen::AngleAxisf (PI_angle, Eigen::Vector3f::UnitZ()));
    initial_guess = transform.matrix();

    for(int tras = 0;tras<=3;tras++){
      time_t current_time;
      current_time = time(NULL);

      config_position_z = position_z + tras;
      double PI_angle = theta * M_PI/180;
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.translation() << position_x, position_y, config_position_z;
      transform.rotate (Eigen::AngleAxisf (PI_angle, Eigen::Vector3f::UnitZ()));
      initial_guess = transform.matrix();
      pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
      icp.setInputSource(cloud);
      icp.setInputTarget(map_cloud);
      icp.setMaxCorrespondenceDistance(1);  
      icp.setTransformationEpsilon(1e-10); 
      icp.setEuclideanFitnessEpsilon(0.0000001); 
      icp.setMaximumIterations(100);   
      icp.align(*output, initial_guess);
      score = icp.getFitnessScore(0.5);
      if(score <= smallest_score){
        smallest_score = score;
        smallest_tras = tras;
        smallest_theta = theta;
        transformation = icp.getFinalTransformation().cast<double>();
      }
      cout << "time = " << time(NULL) - current_time  << endl;
      // cout << "score = " << score << endl;
      // cout << "smallest_score = " << smallest_score << endl;
      // cout << "tras = " << tras <<endl;
      // cout << "smallest_theta = " << smallest_theta << endl;
      // cout  << "----------------" <<endl;
      }
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(*output,ros_msg);
    ros_msg.header.frame_id = "map";
    pub_lidar_icped.publish(ros_msg);
    }
  }

// ======= After first scan, we use last scan ego-pose as initial guess ==========
if(fre > 1){
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(cloud);
  icp.setInputTarget(map_cloud);
  icp.setMaxCorrespondenceDistance(1);  
  icp.setTransformationEpsilon(1e-10); 
  icp.setEuclideanFitnessEpsilon(0.001); 
  icp.setMaximumIterations(100);   
  icp.align(*output, initial_guess);
  transformation = icp.getFinalTransformation().cast<double>();
}


Eigen::Matrix4d car_pose = transformation * tf_transformation.inverse();
Eigen::Affine3d transform_c2l, transform_m2l;
transform_m2l.matrix() = transformation.cast<double>();
Eigen::Affine3d tfff;
tfff.matrix() = tf_transformation;
Eigen::Affine3d t_p; //=  transform_m2l * tfff.inverse();
t_p = transform_m2l * tfff.inverse();
tf::Transform transform_tf;
tf::transformEigenToTF(t_p, transform_tf);
tf::Quaternion qq = transform_tf.getRotation();
tfScalar yaw1, pitch1, roll1;
tf::Matrix3x3 mat(qq);
mat.getEulerYPR(yaw1, pitch1, roll1);
cout  << msg->header.stamp << "," << t_p.translation().x() << "," << t_p.translation().y() << "," << t_p.translation().z() << "," << yaw1 << "," << pitch1 << "," << roll1 << endl;

static tf::TransformBroadcaster br;
br.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), "map", "velodyne"));

/*ofstream outFile;
  outFile.open("/data/david/david_ws/src/sdc_localization/8_easy_3.csv.csv", ios::app); //
  outFile  << msg->header.stamp << "," << t_p.translation().x() << "," << t_p.translation().y() << "," << t_p.translation().z() << "," << yaw1 << "," << pitch1 << "," << roll1 << endl;
  //outFile << msg->header.stamp << "," << x << "," << y << "," << z << "," << yaw << "," << pitch << "," << roll << endl;
  outFile.close();*/
visualization_msgs::Marker marker;
marker.header.frame_id = "velodyne";
marker.header.stamp = ros::Time();
marker.ns = "position";
marker.id = fre;
marker.type = visualization_msgs::Marker::SPHERE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 0;
marker.pose.position.y = 0;
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.5;
marker.scale.y = 0.5;
marker.scale.z = 0.5;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
pub_position.publish(marker);

sensor_msgs::PointCloud2 ros_msg;
pcl::toROSMsg(*output,ros_msg);
ros_msg.header.frame_id = "map";
pub_lidar_icped.publish(ros_msg);


initial_guess = transformation.cast<float>();

cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
output.reset(new pcl::PointCloud<pcl::PointXYZI>);

transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

fre+=1;
cout << "fre: "<< fre << endl;
}

void SubscribeAndPublish::map_callback(const sensor_msgs::PointCloud2ConstPtr& map_msg){
  //pcl::PointCloud<pcl::PointXYZI> map;
  pcl::fromROSMsg (*map_msg, *map_cloud);//cloud is the output

}
void SubscribeAndPublish::gps_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
  
  gps_point = *msg;
  //&gps_point = msg
  if(gps_time==1){
    position_x = gps_point.point.x;
    position_y = gps_point.point.y;
    position_z = gps_point.point.z;

  }
  gps_time++;
}

void SubscribeAndPublish::tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg){
  

  double x = msg->transforms[0].transform.translation.x;
  double y = msg->transforms[0].transform.translation.y;
  double z = msg->transforms[0].transform.translation.z;
  double r_x = msg->transforms[0].transform.rotation.x;
  double r_y = msg->transforms[0].transform.rotation.y;
  double r_z = msg->transforms[0].transform.rotation.z;
  double r_w = msg->transforms[0].transform.rotation.w;



  Eigen::Quaterniond quaternion(r_w,r_x,r_y,r_z);
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = quaternion.toRotationMatrix();
  tf_transformation.setIdentity();
  tf_transformation.block<3,3>(0,0) = rotation_matrix ;
  Eigen::Vector3d translation(x,y,z);
  tf_transformation.block<3,1>(0,3) = translation ;


  //tf::Transform tf_transform_tf;
  //tf::transformEigenToTF(tf_transformation, transfotf_transform_tfrm_tf);

}

void SubscribeAndPublish::imu_callback(const sensor_msgs::Imu::ConstPtr& msg3)
{
  double now_time = msg3->header.stamp.toSec();    //now_time    
  

 
  double ow = msg3->orientation.w;
  double ox = msg3->orientation.x;  
  double oy = msg3->orientation.y;         //build linear_acceleration --> ab
  double oz = msg3->orientation.z;
  Quaterniond q(ow, ox, oy, oz);
  Matrix3d rotation_matrix = q.toRotationMatrix();
  
  imu_eulerAngle = rotation_matrix.eulerAngles(2,1,0);
  //cout << "yaw pitch roll = " << imu_eulerAngle.transpose()  << endl; //* (180 / M_PI)
  

  

}