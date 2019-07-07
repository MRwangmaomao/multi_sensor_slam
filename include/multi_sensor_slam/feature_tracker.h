#pragma once
#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "multi_sensor_slam/parameter.h"
#include "multi_sensor_slam/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
	FeatureTracker();

	// void readImage(float time, cv::Mat image0,  cv::Mat image1);
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void addPoints();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    int row_, col_;
    cv::Mat imTrack_;
    cv::Mat mask_;
    cv::Mat fisheye_mask_;
    cv::Mat prev_img_, cur_img_;
    vector<cv::Point2f> n_pts_;
    vector<cv::Point2f> predict_pts_;
    vector<cv::Point2f> predict_pts_debug_;
    vector<cv::Point2f> prev_pts_, cur_pts_, cur_right_pts_;
    vector<cv::Point2f> prev_un_pts_, cur_un_pts_, cur_un_right_pts_;
    vector<cv::Point2f> pts_velocity_, right_pts_velocity_;
    vector<int> ids_, ids_right_;
    vector<int> track_cnt_;
    map<int, cv::Point2f> cur_un_pts_map_, prev_un_pts_map_;
    map<int, cv::Point2f> cur_un_right_pts_map_, prev_un_right_pts_map_;
    map<int, cv::Point2f> prevLeftPtsMap_;
    vector<camodocal::CameraPtr> m_camera_;
    double cur_time_;
    double prev_time_;
    bool stereo_cam_;
    int n_id_;
    bool hasPrediction_;
	
	virtual ~FeatureTracker();
};
