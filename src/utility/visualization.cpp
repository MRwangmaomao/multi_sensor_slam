#include "multi_sensor_slam/visualization.h"
 
using namespace ros;
using namespace Eigen;

Publisher pub_odometry;
Publisher pub_path;
Publisher pub_image_track;

void registerPub(ros::NodeHandle &n)
{
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);
}


void pubOdometry(nav_msgs::Odometry odometry, const std_msgs::Header &header)
{
    // if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    // {
    //     nav_msgs::Odometry odometry;
    //     odometry.header = header;
    //     odometry.header.frame_id = "world";
    //     odometry.child_frame_id = "world";
    //     Quaterniond tmp_Q;
    //     tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
    //     odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
    //     odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
    //     odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
    //     odometry.pose.pose.orientation.x = tmp_Q.x();
    //     odometry.pose.pose.orientation.y = tmp_Q.y();
    //     odometry.pose.pose.orientation.z = tmp_Q.z();
    //     odometry.pose.pose.orientation.w = tmp_Q.w();
    //     odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    //     odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    //     odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    //     pub_odometry.publish(odometry);

    //     geometry_msgs::PoseStamped pose_stamped;
    //     pose_stamped.header = header;
    //     pose_stamped.header.frame_id = "world";
    //     pose_stamped.pose = odometry.pose.pose;
    //     path.header = header;
    //     path.header.frame_id = "world";
    //     path.poses.push_back(pose_stamped);
    //     pub_path.publish(path);

    //     // write result to file
    //     ofstream foutC(VINS_RESULT_PATH, ios::app);
    //     foutC.setf(ios::fixed, ios::floatfield);
    //     foutC.precision(0);
    //     foutC << header.stamp.toSec() * 1e9 << ",";
    //     foutC.precision(5);
    //     foutC << estimator.Ps[WINDOW_SIZE].x() << ","
    //             << estimator.Ps[WINDOW_SIZE].y() << ","
    //             << estimator.Ps[WINDOW_SIZE].z() << ","
    //             << tmp_Q.w() << ","
    //             << tmp_Q.x() << ","
    //             << tmp_Q.y() << ","
    //             << tmp_Q.z() << ","
    //             << estimator.Vs[WINDOW_SIZE].x() << ","
    //             << estimator.Vs[WINDOW_SIZE].y() << ","
    //             << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
    //     foutC.close();
    //     Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
    //     printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
    //                                                         tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    // }
}

void pubTrackImage(const cv::Mat &imgTrack, const double time)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(time);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track.publish(imgTrackMsg);
}