
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string metricFile;
string trajFile;
string mapFile;
string globalGraphSizeFile;

double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.3;
double exploredVolumeVoxelSize = 0.5;
double transInterval = 0.2;
double yawInterval = 10.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;
int globalGraphSize = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudrefined(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZ> exploredVolumeDwzFilter;

sensor_msgs::PointCloud2 overallMap2;

ros::Publisher *pubExploredAreaPtr = NULL;
ros::Publisher *pubTrajectoryPtr = NULL;
ros::Publisher *pubExploredVolumePtr = NULL;
ros::Publisher *pubTravelingDisPtr = NULL;
ros::Publisher *pubTimeDurationPtr = NULL;
ros::Publisher *pubRuntimePtr = NULL;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;
FILE *globalGraphSizeFilePtr = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  // It takes the time and converts it from a ros object to a double number
  systemTime = odom->header.stamp.toSec();
  
  // Translate orientation quaternion in Euler angles: roll, pitch, yaw
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  // Compute the diffetence btw the new measure yaw angle and the last measured and "normalize" the difference
  // to line btw 0 and PI
  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;
  
  // Compute the difference btw the odometry position and the last robot position measured and compute the distance the
  // robot moved
  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  // Until the system is initialized it takes the robot state, which will be the initial reference once finished the initialization
  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  // If the system is completely initialized it keeps trakc of the time elapsed and publishes it
  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
    
    std_msgs::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  // If the variation of translation and rotation is smaller than a threshold do nothing
  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }

  // If the system is not initialized set the initial distance to 0, the initial time to 0 and define the system as initialized
  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  // Increase the cumulative travelled distance
  travelingDis += dis;

  // Take the last robot pose
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);
  
  // Create a point corresponding to the robot position + travelled distance and add it to the trajectory point cloud
  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  // Converts the trajectory point cloud into a compatible ROS message and publishes it
  sensor_msgs::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "world";
  pubTrajectoryPtr->publish(trajectory2);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  // If the system is still not initialized increase the counter
  if (!systemDelayInited) {
    systemDelayCount++;
    // If the counter is higher than a threshold(5) set the system initialized
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  // If the system is not initalized do nothing
  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  // Convert the incoming ros message into a pointcloud object
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  // Increase the explored volume cloud adding the last measurement
  *exploredVolumeCloud += *laserCloud;

  // Perform a dowsample, considering for instace a voxel of 0.2m, all the points within a voxel are considered as a single point
  // to avoid point cloud exploding
  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  // Pointer swap
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  // Compute the amount of volume explored multipling the voxel volume by the number of voxels
  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "world";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  // runtime=0;
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

void laserCloudHandlerB1(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  laserCloudrefined->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudrefined(new pcl::PointCloud<pcl::PointXYZ>());

  float offset_b1_x = 0.0;
  float offset_b1_y = -2.0;
  float offset_b1_z = 0.0;

  pcl::PointXYZ point;
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      // geometry_msgs::Point cloudpoint;
      // tf::Vector3 cloudpoint(point.x,point.y,point.z);
      // tf::Vector3 cloudpoint_tf_robot = cloud_to_robot_transform * cloudpoint;
      // tf::pointTFToMsg(carrot_point_tf_robot, carrot_waypoint_robot.position);

      point.x = point.x + offset_b1_x; // cloudpoint_tf_robot[0];
      point.y = point.y + offset_b1_y; // cloudpoint_tf_robot[1];
      point.z = point.z + offset_b1_z; //cloudpoint_tf_robot[2];

      
      laserCloudrefined->push_back(point);
    
    }
  laserCloud->clear();

  for (int i = 0; i < laserCloudrefined->points.size(); i++) {

    laserCloudrefined->push_back(laserCloudrefined->points[i]);  
  }

  // laserCloud->setInputCloud(laserCloudrefined);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "world";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }
  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  // runtime=0;
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}


void laserCloudHandlerB2(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  laserCloudrefined->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);


  float offset_b2_x = 0.0;
  float offset_b2_y = -4.0;
  float offset_b2_z = 0.0;

  pcl::PointXYZ point;
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      // geometry_msgs::Point cloudpoint;
      // tf::Vector3 cloudpoint(point.x,point.y,point.z);
      // tf::Vector3 cloudpoint_tf_robot = cloud_to_robot_transform * cloudpoint;
      // tf::pointTFToMsg(carrot_point_tf_robot, carrot_waypoint_robot.position);

      point.x = point.x + offset_b2_x; // cloudpoint_tf_robot[0];
      point.y = point.y + offset_b2_y; // cloudpoint_tf_robot[1];
      point.z = point.z + offset_b2_z; //cloudpoint_tf_robot[2];

      
      laserCloudrefined->push_back(point);
    
    }
  laserCloud->clear();

  for (int i = 0; i < laserCloudrefined->points.size(); i++) {

    laserCloudrefined->push_back(laserCloudrefined->points[i]);  
  }

  // laserCloud->setInputCloud(laserCloudrefined);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "world";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  // runtime=0;
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}


void runtimeHandler(const std_msgs::Float32::ConstPtr& runtimeIn)
{
  runtime = runtimeIn->data;
  
  // Added by me
  std_msgs::Float32 runtimeMsg;
  runtimeMsg.data = runtime;

  // pubRuntimePtr->publish(runtimeMsg); // Added by me
}

void gbplannerruntimeHandler(const std_msgs::Float32MultiArray::ConstPtr& runtimeIn)
{
  runtime=0;
  for(int i =0; i < runtimeIn->data.size(); ++i){
    runtime += runtimeIn->data[i];
  }
  runtime = runtimeIn->data[0]+runtimeIn->data[1]+runtimeIn->data[2]+runtimeIn->data[3];

  // Added by me
  std_msgs::Float32 runtimeMsg;
  runtimeMsg.data = runtime;

  // pubRuntimePtr->publish(runtimeMsg); // Added by me
}

void globalGraphSizeCallback(const std_msgs::Int32 msg){
  fprintf(globalGraphSizeFilePtr, "%d\n", msg.data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualizationTools");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("metricFile", metricFile);
  nhPrivate.getParam("trajFile", trajFile);
  nhPrivate.getParam("mapFile", mapFile);
  nhPrivate.getParam("globalGraphSizeFile", globalGraphSizeFile);
  nhPrivate.getParam("overallMapVoxelSize", overallMapVoxelSize);
  nhPrivate.getParam("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nhPrivate.getParam("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nhPrivate.getParam("transInterval", transInterval);
  nhPrivate.getParam("yawInterval", yawInterval);
  nhPrivate.getParam("overallMapDisplayInterval", overallMapDisplayInterval);
  nhPrivate.getParam("exploredAreaDisplayInterval", exploredAreaDisplayInterval);

  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("registered_scan", 5, laserCloudHandler);


  ros::Subscriber subLaserCloudB1 = nh.subscribe<sensor_msgs::PointCloud2> ("b1_scan", 5, laserCloudHandlerB1);

  ros::Subscriber subLaserCloudB2 = nh.subscribe<sensor_msgs::PointCloud2> ("b2_scan", 5, laserCloudHandlerB2);


  ros::Subscriber subRuntime = nh.subscribe<std_msgs::Float32> ("runtime", 5, runtimeHandler);

  ros::Subscriber subgbRuntime = nh.subscribe<std_msgs::Float32MultiArray> ("gbp_time_log", 5, gbplannerruntimeHandler);

  ros::Subscriber globalGraphSizeSub = nh.subscribe<std_msgs::Int32> ("global_graph_size", 5, globalGraphSizeCallback);

  ros::Publisher pubOverallMap = nh.advertise<sensor_msgs::PointCloud2> ("overall_map", 5);

  ros::Publisher pubExploredArea = nh.advertise<sensor_msgs::PointCloud2> ("explored_areas", 5);
  pubExploredAreaPtr = &pubExploredArea;

  ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::PointCloud2> ("trajectory", 5);
  pubTrajectoryPtr = &pubTrajectory;

  ros::Publisher pubExploredVolume = nh.advertise<std_msgs::Float32> ("explored_volume", 5);
  pubExploredVolumePtr = &pubExploredVolume;

  ros::Publisher pubTravelingDis = nh.advertise<std_msgs::Float32> ("traveling_distance", 5);
  pubTravelingDisPtr = &pubTravelingDis;

  ros::Publisher pubTimeDuration = nh.advertise<std_msgs::Float32> ("time_duration", 5);
  pubTimeDurationPtr = &pubTimeDuration;

  ros::Publisher pubstop = nh.advertise<std_msgs::Int8> ("stop", 5);
  
  // ros::Publisher pubRuntime = nh.advertise<std_msgs::Float32> ("runtime", 5);
  // pubRuntimePtr = &pubRuntime;

  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  // pcl::PLYReader ply_reader;
  // if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
  //   printf("\nCouldn't read pointcloud.ply file.\n\n");
  // }

  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();

  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  // metricFile += "_" + timeString + ".txt";
  // trajFile += "_" + timeString + ".txt";
  // globalGraphSizeFile += "_" + timeString + ".txt";
  metricFile += "_run_1.txt";
  trajFile += "_run_1.txt";
  globalGraphSizeFile += "_run_1.txt";
  metricFilePtr = fopen(metricFile.c_str(), "w");
  trajFilePtr = fopen(trajFile.c_str(), "w");
  globalGraphSizeFilePtr = fopen(globalGraphSizeFile.c_str(), "w");

  if (metricFilePtr == NULL) {
  ROS_ERROR("Cannot open metric file: %s", metricFile.c_str());
  }

  if (trajFilePtr == NULL) {
    ROS_ERROR("Cannot open trajectory file: %s", trajFile.c_str());
  }

  ROS_INFO("Metric file path: %s", metricFile.c_str());
  ROS_INFO("Trajectory file path: %s", trajFile.c_str());

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      overallMap2.header.stamp = ros::Time().fromSec(systemTime);
      overallMap2.header.frame_id = "world";
      pubOverallMap.publish(overallMap2);

      overallMapDisplayCount = 0;
    }

    // if(timeDuration > 1800){
    //   std_msgs::Int8 stop_msg;
    //   stop_msg.data = 1;
    //   pubstop.publish(stop_msg);
    // }

    status = ros::ok();
    rate.sleep();
  }

  // pcl::PLYWriter ply_writer;
  // ply_writer.write(mapFile, *exploredAreaCloud, true, false);

  fclose(metricFilePtr);
  fclose(trajFilePtr);
  fclose(globalGraphSizeFilePtr);

  printf("\nExploration metrics and vehicle trajectory are saved in 'src/visualization_tools/log'.\n\n");
  
  return 0;
}
