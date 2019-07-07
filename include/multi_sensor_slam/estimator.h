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
	void getOdom(float left_rear_wheel_num, float right_rear_pulses_num, double current_time);
	void inputImage(double time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    

	FeatureTracker feature_tracker_;
	int frame_count_;
	int sum_of_outlier_, sum_of_back_, sum_of_front_, sum_of_invalid_;
	int input_imageCnt_;

	std::mutex mProcess_;
    std::mutex mBuf_;
	queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf_;
	
	WheelOdometer wheel_odometer_;
};
