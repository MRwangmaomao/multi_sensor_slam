#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "multi_sensor_slam/parameter.h" 
#include "multi_sensor_slam/feature_tracker.h"
#include "multi_sensor_slam/wheel_odometer.h"
#include "multi_sensor_slam/visualization.h"


class Estimator
{
public:
	Estimator();
	virtual ~Estimator(); 
	void setParameter();
	void get_odom(float left_rear_wheel_num, float right_rear_pulses_num, double current_time);
	void inputImage(double time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    FeatureTracker featureTracker;


	int frame_count;
	int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
	int inputImageCnt;

	std::mutex mProcess;
    std::mutex mBuf;
	queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
	
	WheelOdometer wheel_odometer_;
};