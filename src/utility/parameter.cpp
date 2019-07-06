
#include "multi_sensor_slam/parameter.h" 
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
 
std::string IMU_TOPIC;
int ROW, COL;
int NUM_OF_CAM; 
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

int USE_IMU;
int MULTIPLE_THREAD;

std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::vector<std::string> CAM_NAMES;

std::vector<Eigen::Matrix3d> RIC_Front;
std::vector<Eigen::Vector3d> TIC_Front;
std::vector<Eigen::Matrix3d> RIC_Rear;
std::vector<Eigen::Vector3d> TIC_Rear;

std::string DWS_TOPIC;
double Front_Wheel_Radius;
double Rear_wheel_Radius;
double Wheel_Base;
double Front_Track;
double Rear_Track;
int Front_Wheel_Pulses;
int Rear_Wheel_Pulses;
double Left_Front_Speed_Sf;
double Right_Front_Speed_Sf;
double Left_Rear_Speed_Sf;
double Right_Rear_Speed_Sf;

void readParameters(std::string config_file)
{
    FILE * fh = fopen(config_file.c_str(), "r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;

    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    fsSettings["dws_topic"] >> DWS_TOPIC;

    Front_Wheel_Radius = fsSettings["front_wheel_radius"];
    Rear_wheel_Radius = fsSettings["rear_wheel_radius"];
    Wheel_Base = fsSettings["wheel_base"];
    Front_Track = fsSettings["front_track"];
    Rear_Track = fsSettings["rear_track"];
    Front_Wheel_Pulses = fsSettings["front_wheel_pulses"];
    Rear_Wheel_Pulses = fsSettings["rear_wheel_pulses"];
    Left_Front_Speed_Sf = fsSettings["left_front_speed_sf"];
    Right_Front_Speed_Sf = fsSettings["right_front_speed_sf"];
    Left_Rear_Speed_Sf = fsSettings["left_rear_speed_sf"];
    Right_Rear_Speed_Sf = fsSettings["right_rear_speed_sf"];
    cv::Mat Tcv_1, Tcv_2;
    fsSettings["dr_T_rear_camera"] >> Tcv_1;
    fsSettings["dr_T_front_camera"] >> Tcv_2;
    Eigen::Matrix4d T;
    cv::cv2eigen(Tcv_1, T);
    RIC_Front.push_back(T.block<3, 3>(0, 0));
    TIC_Front.push_back(T.block<3, 1>(0, 3)); 
    cv::cv2eigen(Tcv_2, T);
    RIC_Rear.push_back(T.block<3, 3>(0, 0));
    TIC_Rear.push_back(T.block<3, 1>(0, 3));

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    std::cout << cam0Path << std::endl;
    CAM_NAMES.push_back(cam0Path);


 
    fsSettings.release();
}

