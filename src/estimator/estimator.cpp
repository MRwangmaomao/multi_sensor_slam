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
    feature_tracker_.readIntrinsicParameter(CAM_NAMES);
}

/**
 * @brief 
 * 
 * @param left_rear_wheel_num 
 * @param right_rear_pulses_num 
 */
void Estimator::getOdom(float left_rear_wheel_num, float right_rear_pulses_num, double current_time)
{
    wheel_odometer_.calculateOdom(left_rear_wheel_num, right_rear_pulses_num, current_time);
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(wheel_odometer_.last_time_);
    // pubOdometry(*this, header);
    
}

void Estimator::inputImage(double time, const cv::Mat &_img, const cv::Mat &_img1) // 图像入口
{
    input_imageCnt_++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;

    if(_img1.empty())
        featureFrame = feature_tracker_.trackImage(time, _img);
    else
        featureFrame = feature_tracker_.trackImage(time, _img, _img1);

    if(SHOW_TRACK)
    {
        cv::Mat imgTrack = feature_tracker_.getTrackImage();
        pubTrackImage(imgTrack, time);
    }

    // if(MULTIPLE_THREAD)
    // {
    //     if(input_imageCnt_    % 2 == 0)
    //     {
    //         mBuf_.lock();
    //         featureBuf_.push(make_pair(time, featureFrame));
    //         mBuf_.unlock();
    //     }
    // }
    else
    {
        mBuf_.lock();
        featureBuf_.push(make_pair(time, featureFrame));
        mBuf_.unlock();
        TicToc processTime;
        printf("process time: %f\n", processTime);
    }
    
}