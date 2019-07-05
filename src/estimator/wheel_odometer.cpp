#include "multi_sensor_slam/wheel_odometer.h"


WheelOdometer::WheelOdometer()
{ 
    last_left_rear_pulses_num_ = 0;
    last_right_rear_pulses_num_ = 0;
    yaw = 0.0;
}

WheelOdometer::~WheelOdometer()
{
}

/**
 * @brief 
 * 
 * @param rear_wheel_radius 
 * @param wheel_base 
 * @param rear_track 
 * @param rear_wheel_pulses 
 * @param left_rear_speed_sf 
 * @param right_rear_speed_sf 
 */
void WheelOdometer::setWheelParam(double rear_wheel_radius, double wheel_base, double rear_track, double rear_wheel_pulses, double left_rear_speed_sf, double right_rear_speed_sf)
{
    rear_wheel_radius_ = rear_wheel_radius;
    wheel_base_ = wheel_base;
    rear_track_ = rear_track;
    rear_wheel_pulses_ = rear_wheel_pulses;
    left_rear_speed_sf_ = left_rear_speed_sf;
    right_rear_speed_sf_ = right_rear_speed_sf;
}

/**
 * @brief 
 * 
 * @param left_rear_wheel_num 
 * @param right_rear_pulses_num 
 */
void WheelOdometer::calculate_odom(float left_rear_wheel_num, float right_rear_pulses_num, double current_time)
{
    float increase_left = 0;
    float increase_right = 0;

    if(last_left_rear_pulses_num_ != 0 && last_right_rear_pulses_num_ != 0)
    {  
        float increase_left = (left_rear_wheel_num - last_left_rear_pulses_num_)/rear_wheel_pulses_ * rear_wheel_radius_ * M_PI * 2.0;
        float increase_right = (right_rear_pulses_num - last_right_rear_pulses_num_) /rear_wheel_pulses_ * rear_wheel_radius_ * M_PI * 2.0;
        float dt = current_time - last_time_;
        float dx = (increase_left + increase_right)/2;
        double dtheta = (increase_right -increase_left) / rear_track_;
        yaw += dtheta;
        T(0) += dx * cos(yaw); 
        T(1) += dx * sin(yaw);
        std::cout << T << std::endl;
    } 
    last_time_ = current_time;
    last_left_rear_pulses_num_ = left_rear_wheel_num;
    last_right_rear_pulses_num_ = right_rear_pulses_num;
}