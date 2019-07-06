#include "multi_sensor_slam/estimator.h"

Estimator::Estimator()
{
    
}

Estimator::~Estimator()
{
}

/**
 * @brief 
 * 
 */
void Estimator::setParameter()
{
    wheel_odometer_.setWheelParam(Rear_wheel_Radius, Wheel_Base, Rear_Track, Rear_Wheel_Pulses, Left_Rear_Speed_Sf, Right_Rear_Speed_Sf);
}

/**
 * @brief 
 * 
 * @param left_rear_wheel_num 
 * @param right_rear_pulses_num 
 */
void Estimator::get_odom(float left_rear_wheel_num, float right_rear_pulses_num, double current_time)
{
    wheel_odometer_.calculate_odom(left_rear_wheel_num, right_rear_pulses_num, current_time);
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(wheel_odometer_.last_time_);
    // pubOdometry(*this, header);
    
}

void Estimator::inputImage(float time, cv::Mat image0,  cv::Mat image1) // 图像入口
{
    
}