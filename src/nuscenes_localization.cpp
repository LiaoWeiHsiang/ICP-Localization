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
#include <pcl/filters/conditional_removal.h>
#include <algorithm>
#include <string.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/crop_box.h>



using namespace Eigen;
using namespace std;

class SubscribeAndPublish  
{  

    
    public:  
      SubscribeAndPublish()  
      {  
        
        sub_raw_imu = nh.subscribe("/imu/data", 0.3, &SubscribeAndPublish::imu_callback, this);
        
        
        sub_lidar = nh.subscribe("/lidar_points", 10000, &SubscribeAndPublish::lidar_callback, this);
        pub_map = nh.advertise<sensor_msgs::PointCloud2> ("map", 1, true);
        sub_map = nh.subscribe("/map", 10000, &SubscribeAndPublish::map_callback, this);
        sub_gps = nh.subscribe("/fix", 10000, &SubscribeAndPublish::gps_callback, this);
        pub_lidar_icped = nh.advertise<sensor_msgs::PointCloud2> ("lidar_after_icp", 1);
        sub_tf = nh.subscribe("/tf", 10000, &SubscribeAndPublish::tf_callback, this);

        pub_croped_lidar = nh.advertise<sensor_msgs::PointCloud2> ("/croped_lidar", 1, true);


        
        

      }  

    private:  
      ros::NodeHandle nh;   
      ros::Publisher pub_imu;
      ros::Publisher pub_combine_marker;  
      ros::Publisher pub_zed_odom_marker; 
      ros::Publisher pub_imu_marker; 
      ros::Publisher pub_map;
      ros::Subscriber sub_raw_imu; 
      ros::Subscriber sub_zed_odom;
      ros::Publisher pub_croped_lidar; 

      
      ros::Subscriber sub_gps; 
      ros::Subscriber sub_lidar;
      ros::Subscriber sub_map;
      ros::Publisher pub_lidar_icped;
      ros::Subscriber sub_tf;

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg3); 
    /*void callback2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); 
    void callback3(const nav_msgs::Odometry::ConstPtr& msg2); */
    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg); 
    void gps_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);

    void map_callback(const sensor_msgs::PointCloud2ConstPtr& map_msg); 
    



};


int fre = 1;
float last_time;

Eigen::Matrix4d tf_transformation;
int theta;
double position_x;
double position_y;
double position_z;
int gps_time=0;
Eigen::Vector3d imu_eulerAngle;
Eigen::Matrix4d transformation;
Eigen::Matrix4f initial_guess;

//Eigen::Vector4f gps_info;
double gps_info[10][4];
//vector<double> gps_info[30];

pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
geometry_msgs::PointStamped gps_point;


int main(int argc, char** argv)
{
  
  

  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  
    

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
//float get_max(vector<double> part_matrix);
double get_max(vector<double> part_matrix){
double max_num = part_matrix[0];
for(int i= 0;i<part_matrix.size();i+=1){
if(max_num<part_matrix[i]){
  max_num = part_matrix[i];
}
}
//cout << part_matrix[0] << endl;
return max_num;
}

double get_min(vector<double> part_matrix){
double min_num = part_matrix[0];
for(int i= 0;i<part_matrix.size();i+=1){
if(min_num > part_matrix[i]){
  min_num = part_matrix[i];
}
}
//cout << part_matrix[0] << endl;
return min_num;
}

void SubscribeAndPublish::lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  cout << "----------------- = " << endl;
  cout << "fre = " << fre << endl;
cout << "ros_time = " << msg->header.stamp << endl;
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZI>::Ptr downsample_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *transformed_cloud);//cloud is the output


 

//***************read_bbox***************
vector<double> matrix;
fstream file;
int aaa = fre-1;
string aaaa = to_string(aaa);
//char const *aaaaa = aaaa.c_str();
string bbox_path = "/data/david/david_ws/src/sdc_localization/bbox/p3/" + aaaa + ".csv";  //%c.csv",aaaaa;

  //file.open("/data/david/david_ws/src/sdc_localization/bbox/p2/0.csv");
  file.open(bbox_path);

  //getdir(map_path, files);


  string line;
  //vector<vector> matrix2;
  while (getline( file, line,'\n')) 
  {
    
    istringstream templine(line); 
    string data;
    while (getline( templine, data,' ')) 
    {
      matrix.push_back(atof(data.c_str()));
    }
    //matrix2.push_back(matrix)
    //matrix.clear();
  }
  file.close();

  cout << "matrix = " << matrix[9] <<endl;
//******************************************




//************remove_box***************



pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>);

for(int bbox_num=0; bbox_num <(matrix.size()/24) ;bbox_num+=1){ //(matrix.size()/24)-20
if(bbox_num==0){
  cloudIn = transformed_cloud;
}  

vector<double> x_part_of_matrix(matrix.begin()+24*bbox_num,matrix.begin()+7+24*bbox_num);
vector<double> y_part_of_matrix(matrix.begin()+8+24*bbox_num,matrix.begin()+15+24*bbox_num);
vector<double> z_part_of_matrix(matrix.begin()+16+24*bbox_num,matrix.begin()+23+24*bbox_num);
double x_max_num = get_max(x_part_of_matrix);
double y_max_num = get_max(y_part_of_matrix);
double z_max_num = get_max(z_part_of_matrix);

double x_min_num = get_min(x_part_of_matrix);
double y_min_num = get_min(y_part_of_matrix);
double z_min_num = get_min(z_part_of_matrix);




  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloudIn);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_min_num, z_max_num);
  pass_z.setFilterLimitsNegative(true);
  pass_z.filter(*cloudOut);

  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloudOut);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_min_num, x_max_num);
  pass_x.setFilterLimitsNegative(true);
  pass_x.filter(*cloudOut);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloudOut);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_min_num, y_max_num);
  pass_y.setFilterLimitsNegative(true);
  pass_y.filter(*cloudOut);





cloudIn = cloudOut;
//cloudOut = cloudIn;
}



//*******************************************************
sensor_msgs::PointCloud2 cloudOut_ros;
pcl::toROSMsg (*cloudOut, cloudOut_ros);
cloudOut_ros.header.stamp = ros::Time::now();
cloudOut_ros.header.frame_id = "map";
pub_croped_lidar.publish(cloudOut_ros);
//pub_croped_lidar





//**************read_map**************
/*const string package_path = ros::package::getPath;
cout << "package_path = " << package_path << endl;*/
pcl::PointCloud<pcl::PointXYZ>::Ptr total_pcd_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr single_pcd__ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointIndicesPtr map_ground (new pcl::PointIndices);
if(fre==1){
double lidar_time = msg->header.stamp.toSec();
for(int o=0;o<=3;o++){
  //if(gps_info[o][0] == lidar_time)
    cout << gps_info[o][0] << endl;
}

int map_x = (position_x / 100);
map_x = map_x *100;
string str_map_x = to_string(map_x);
char const *char_map_x = str_map_x.c_str();  //use char const* as target type
int map_y = (position_y / 100);
map_y = map_y *100;
string str_map_y = to_string(map_y);
char const *char_map_y = str_map_y.c_str();




cout << "map_x = " << map_x <<endl;
  int getdir(string dir, vector<string> &files);
  string map_path = string("/data/david/david_ws/src/sdc_localization/map");
  vector<string> files = vector<string>();
  
  getdir(map_path, files);

for (int map_x_range = map_x-200;map_x_range <= map_x + 200;map_x_range+=100){
  string str_map_x = to_string(map_x_range);
  char const *char_map_x = str_map_x.c_str();  //use char const* as target type
  for (int map_y_range = map_y-200;map_y_range <= map_y + 200;map_y_range+=100){
    string str_map_y = to_string(map_y_range);
  char const *char_map_y = str_map_y.c_str();  //use char const* as target type

for(int i=2; i<files.size(); i++){  

  const char* char_file = files[i].c_str();
  const char *x_name_match = strstr(char_file, char_map_x);

  if(x_name_match!=NULL && strlen(x_name_match)>=10){//
    //cout << "char_map_y" << char_map_y <<endl;
    const char *y_name_match = strstr(x_name_match, char_map_y);
    if(y_name_match!=NULL){





    cout << "x_name_match = " << x_name_match  << " size = "<< strlen(x_name_match) << endl;

  

    string map_file;
    //sort (files.begin(), files.end());
    //for(int i=2; i<files.size(); i++){    //files.size()
        map_file = map_path + "/"+ "map_" +  x_name_match;
        cout << map_file << endl;
        pcl::io::loadPCDFile<pcl::PointXYZ> (map_file, *single_pcd__ptr);


        //************ remove_map_ground ************
        /*pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> map_pmf;
        map_pmf.setInputCloud (single_pcd__ptr);
        map_pmf.setMaxWindowSize (20);
        map_pmf.setSlope (1.0f);
        map_pmf.setInitialDistance (0.1f);
        map_pmf.setMaxDistance (3.0f);
        map_pmf.extract (map_ground->indices);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> map_extract;
        map_extract.setInputCloud (map_cloud);
        map_extract.setIndices (map_ground);
        map_extract.setNegative(true);
        map_extract.filter (*single_pcd__ptr);*/





        *total_pcd_ptr=*total_pcd_ptr+*single_pcd__ptr;



    //}
}

}
}
}
}



    sensor_msgs::PointCloud2 cloudc_ros;
     pcl::toROSMsg (*total_pcd_ptr, cloudc_ros);
      cloudc_ros.header.stamp = ros::Time::now();
      cloudc_ros.header.frame_id = "map";
      pub_map.publish(cloudc_ros);

    map_cloud = total_pcd_ptr;



  




}



//**********remove_map_ground*************



//************************************
    




//**********************remove_ground*******************


pcl::PointIndicesPtr ground (new pcl::PointIndices);

  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloudOut);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.1f);
  pmf.setMaxDistance (4.3f);
  pmf.extract (ground->indices);

  // Create the filtering ob100ject
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloudOut);
  extract.setIndices (ground);
  extract.setNegative(true);
  extract.filter (*cloud);

//*******************************************************
//*cloud = *cloudOut;

pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);


float smallest_score;

int smallest_tras;
int smallest_theta;
double score;

if(fre==1){


smallest_score = 1000;
cout << "dnfljdsnlkjdsnflkjns" << endl;
cout << "gps = " << "x = " << position_x << "y = " << position_y  << "z = " << position_z <<endl;


//theta=284;



for(int theta=48; theta<=275; theta+=3){ 

  
  double PI_angle = theta * M_PI/180;

cout << "theta = " << theta << endl;

Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//cout << transform.matrix() << endl;
transform.translation() << position_x, position_y, position_z;
transform.rotate (Eigen::AngleAxisf (PI_angle, Eigen::Vector3f::UnitZ()));


initial_guess = transform.matrix();


pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 

icp.setInputSource(cloud);
icp.setInputTarget(map_cloud);
icp.setMaxCorrespondenceDistance(1);  
icp.setTransformationEpsilon(1e-10); 
icp.setEuclideanFitnessEpsilon(0.00000001); 
icp.setMaximumIterations(1000);   
icp.align(*output, initial_guess);


score = icp.getFitnessScore(0.5);
if(score <= smallest_score){
  smallest_score = score;
  smallest_theta = theta;
  transformation = icp.getFinalTransformation().cast<double>();
}
sensor_msgs::PointCloud2 ros_msg;
pcl::toROSMsg(*output,ros_msg);
ros_msg.header.frame_id = "map";
pub_lidar_icped.publish(ros_msg);

cout << "score = " << score << endl;
cout << "smallest_score = " << smallest_score << endl;
cout << "smallest_theta = " << smallest_theta << endl;
cout << "--------------- " << endl;



}



}

if(fre>1){

//for{int  }

//initial_guess = initial_guess

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; 

icp.setInputSource(cloud);
icp.setInputTarget(map_cloud);
icp.setMaxCorrespondenceDistance(1);  
icp.setTransformationEpsilon(1e-10); 
icp.setEuclideanFitnessEpsilon(0.001); 
icp.setMaximumIterations(1000);   
icp.align(*output, initial_guess);
transformation = icp.getFinalTransformation().cast<double>();




}



Eigen::Matrix4d car_pose = transformation * tf_transformation.inverse();


Eigen::Affine3d transform_c2l, transform_m2l;
    transform_m2l.matrix() = transformation.cast<double>();

Eigen::Affine3d tfff;
tfff.matrix() = tf_transformation;

//Eigen::Matrix3f car_pose;// =  transform_m2l * tfff.inverse();

Eigen::Affine3d t_p; //=  transform_m2l * tfff.inverse();
t_p = transform_m2l * tfff.inverse();

//t_p.matrix() = car_pose;
    tf::Transform transform_tf;
    tf::transformEigenToTF(t_p, transform_tf);
    //br.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), "map", "aa"));
    tf::Quaternion qq = transform_tf.getRotation();
    tfScalar yaw1, pitch1, roll1;
    tf::Matrix3x3 mat(qq);
    mat.getEulerYPR(yaw1, pitch1, roll1);
    //Eigen::Vector3d ang = tf_p.rotation().eulerAngles(2, 1, 0);
    cout  << msg->header.stamp << "," << t_p.translation().x() << "," << t_p.translation().y() << "," << t_p.translation().z() << "," << yaw1 << "," << pitch1 << "," << roll1 << endl;



ofstream outFile;
  outFile.open("/data/david/david_ws/src/sdc_localization/8_medium_2.csv", ios::app); // 打开模式可省略
  outFile  << msg->header.stamp << "," << t_p.translation().x() << "," << t_p.translation().y() << "," << t_p.translation().z() << "," << yaw1 << "," << pitch1 << "," << roll1 << endl;
  //outFile << msg->header.stamp << "," << x << "," << y << "," << z << "," << yaw << "," << pitch << "," << roll << endl;
  outFile.close();


sensor_msgs::PointCloud2 ros_msg;
pcl::toROSMsg(*output,ros_msg);
ros_msg.header.frame_id = "map";
pub_lidar_icped.publish(ros_msg);


initial_guess = transformation.cast<float>();




cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
output.reset(new pcl::PointCloud<pcl::PointXYZ>);

transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

fre+=1;
cout << fre << endl;
}

void SubscribeAndPublish::map_callback(const sensor_msgs::PointCloud2ConstPtr& map_msg){
  //pcl::PointCloud<pcl::PointXYZI> map;
  //pcl::fromROSMsg (*map_msg, *map_cloud);//cloud is the output

}
void SubscribeAndPublish::gps_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
  
  gps_point = *msg;
  double ros_time = msg->header.stamp.toSec(); 
  
  if(gps_time==0){

  position_x = gps_point.point.x;
  position_y = gps_point.point.y;
  position_z = gps_point.point.z+2;

}
  //&gps_point = msg

  gps_time+=1;
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



}

void SubscribeAndPublish::imu_callback(const sensor_msgs::Imu::ConstPtr& msg3)
{
  double now_time = msg3->header.stamp.toSec();    //now_time    
  

  //Vector3d now_vg;                    //now velocity
//Vector3d now_sg;                    //now location     

  double ow = msg3->orientation.w;
  double ox = msg3->orientation.x;  
  double oy = msg3->orientation.y;         //build linear_acceleration --> ab
  double oz = msg3->orientation.z;
  Quaterniond q(ow, ox, oy, oz);
  Matrix3d rotation_matrix = q.toRotationMatrix();
  
  imu_eulerAngle = rotation_matrix.eulerAngles(2,1,0);
  //cout << "yaw pitch roll = " << imu_eulerAngle.transpose()  << endl; //* (180 / M_PI)
  

  

}