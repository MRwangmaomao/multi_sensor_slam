#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h> 
#include <eigen3/Eigen/Dense>
#include <cv_bridge/cv_bridge.h>



#include "multi_sensor_slam/estimator.h" 

extern ros::Publisher pub_odometry; 
extern ros::Publisher pub_path;
extern ros::Publisher pub_image_track;

void registerPub(ros::NodeHandle &n);
  
void pubOdometry(nav_msgs::Odometry odometry, const std_msgs::Header &header);

void pubTrackImage(const cv::Mat &imgTrack, const double time);