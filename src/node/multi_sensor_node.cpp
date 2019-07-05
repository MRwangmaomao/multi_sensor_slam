#include <iostream>
#include <vector>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <multi_sensor_slam/dws_info.h> 
#include <opencv2/opencv.hpp>
#include "multi_sensor_slam/parameter.h" 
#include "multi_sensor_slam/estimator.h" 
#include "multi_sensor_slam/visualization.h"

using namespace Eigen;

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf; 
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;
bool STEREO = true;

/**
 * @brief front camera topic receive callback
 * 
 * @param img_msg 
 */
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

/**
 * @brief 
 * 
 * @param img_msg 
 */
void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

/**
 * @brief Get the Image From Msg object
 * 
 * @param img_msg 
 * @return cv::Mat 
 */
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

/**
 * @brief the IMU Topic Callback
 * 
 * @param imu_msg 
 */
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz); 
    // ROS_DEBUG("imu:%lf", t);
    // return;
}

/**
 * @brief wheel odom pulses callback 
 * 
 * @param dws_msg 
 */
void dws_callback(const multi_sensor_slam::dws_infoConstPtr &dws_msg)
{
    double t = dws_msg->header.stamp.toSec();
    float left_rear_pulse_num = dws_msg->left_rear; 
    float right_rear_pulse_num = dws_msg->right_rear;
    float left_front_pulse_num = dws_msg->left_front;
    float right_front_pulse_num = dws_msg->right_front;  
    estimator.get_odom(left_rear_pulse_num, right_rear_pulse_num, t);
    // ROS_DEBUG("dws:%lf", t);
}

/**
 * @brief extract images with same timestamp from two topics
 * 
 */
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec(); 
                ROS_DEBUG("img0:%lf, img1:%lf", time0, time1);
                if(time0 < time1)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop(); 
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            // if(!image.empty())
            //     estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

/**
 * @brief main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "multi_sensor_slam");
    ros::start();
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // config file
    std::string config_file = "/home/wpr/code/catkin_ws/src/multi_sensor_slam/config/zongmu_car_config.yaml";   
    
    if (!n.getParam("config_file", config_file))
    {
        ROS_INFO("Error: %s\n",config_file.c_str());
        return 1;
    }

    readParameters(config_file);
    estimator.setParameter();
    ROS_INFO("load config_file: %s\n", config_file.c_str());
    
    // receive topic
    ROS_WARN("waiting for image, dws and imu...");

    registerPub(n);
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback); //, ros::TransportHints().tcpNoDelay()
    ros::Subscriber sub_dws = n.subscribe(DWS_TOPIC, 2000, dws_callback); 
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);

    // start a new thread
    std::thread sync_thread{sync_process};
    
    ros::spin();

    return 0;
}