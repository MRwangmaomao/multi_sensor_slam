
#pragma once

#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense> 
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
using namespace std;

extern double INIT_DEPTH; 
extern std::string IMU_TOPIC; 
extern int NUM_OF_CAM; 
extern int USE_IMU; 
extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC; 
extern std::string DWS_TOPIC;
extern double Front_Wheel_Radius;
extern double Rear_wheel_Radius;
extern double Wheel_Base;
extern double Front_Track;
extern double Rear_Track;

extern int Front_Wheel_Pulses;
extern int Rear_Wheel_Pulses;

extern double Left_Front_Speed_Sf;
extern double Right_Front_Speed_Sf;
extern double Left_Rear_Speed_Sf;
extern double Right_Rear_Speed_Sf;


void readParameters(std::string config_file);
