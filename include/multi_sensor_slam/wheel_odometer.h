#pragma once
#include <iostream>
#include <math.h> 
#include <eigen3/Eigen/Dense> 

class WheelOdometer
{
public:
	WheelOdometer();
	
	virtual ~WheelOdometer();
	void setWheelParam(double rear_wheel_radius, double wheel_base, double rear_track, double rear_wheel_pulses, double left_rear_speed_sf, double right_rear_speed_sf);
	void calculate_odom(float left_rear_wheel_num, float right_rear_pulses_num, double current_time); 

	double rear_wheel_radius_;
	double wheel_base_;
	double rear_track_;
	double rear_wheel_pulses_;
	double left_rear_speed_sf_;
	double right_rear_speed_sf_;
	
	float last_left_rear_pulses_num_;
	float last_right_rear_pulses_num_;
	double last_time_;
	
	double yaw;
	Eigen::Vector3d T;
};