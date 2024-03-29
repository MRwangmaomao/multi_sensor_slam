#include "multi_sensor_slam/feature_tracker.h"

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col_ - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row_ - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam_ = 0;
    n_id_ = 0;
    hasPrediction_ = false;
}

/**
 * @brief  设置MASK， 
 * @note   
 * @retval None
 */
void FeatureTracker::setMask()
{
    mask_ = cv::Mat(row_, col_, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts_.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt_[i], make_pair(cur_pts_[i], ids_[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts_.clear();
    ids_.clear();
    track_cnt_.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask_.at<uchar>(it.second.first) == 255)
        {
            cur_pts_.push_back(it.second.first);
            ids_.push_back(it.second.second);
            track_cnt_.push_back(it.first);
            cv::circle(mask_, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

// 把光流没追够的，用corner角点补进去 n_pts + cur_pts == >track_cnt_
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts_)
    {
        cur_pts_.push_back(p);
        ids_.push_back(n_id_++);
        track_cnt_.push_back(1);
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

/**
 * @brief  跟踪点的信息，（7,1） 3+2+2  x y z + u v + vx vy 
 * @note   
 * @param  _cur_time: 
 * @param  &_img: 
 * @param  &_img1: 
 * @retval 
 */
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r; //计时
    cur_time_ = _cur_time;
    cur_img_ = _img;
    row_ = cur_img_.rows;
    col_ = cur_img_.cols;
    cv::Mat rightImg = _img1;
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img_, cur_img_);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts_.clear();

    if (prev_pts_.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction_) // has prediction 少跟踪一些
        {
            cur_pts_ = predict_pts_;

            //computes sparse optical flow using multi-scale Lucas-Kanade algorithm
            //根据prev_img的prev_pts的特征点，在cur_img图像中预测cur_pts
            cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_, cur_pts_, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)//光流跟踪到的特征点
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
               cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_, cur_pts_, status, err, cv::Size(21, 21), 3);
        }
        else // hasPrediction_ = false 多跟踪一些
            cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_, cur_pts_, status, err, cv::Size(21, 21), 3);

        // reverse check 光流逆向追踪， 可能是为了去除outlire, 得到更稳定追踪的点
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts_;
            cv::calcOpticalFlowPyrLK(cur_img_, prev_img_, cur_pts_, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img_, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts_[i], reverse_pts[i]) <= 0.5) //正向和反向都追踪到了，而且距离没有异常
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts_.size()); i++)
            if (status[i] && !inBorder(cur_pts_[i])) // 去除边缘上的点
                status[i] = 0;
        reduceVector(prev_pts_, status);
        reduceVector(cur_pts_, status);
        reduceVector(ids_, status);
        reduceVector(track_cnt_, status);
        //ROS_WARNROS_WARN("temporal optical flow costs: %fms", t_o.toc());
        ROS_WARN("KLT track cnt %d\n", (int)ids_.size());
    }

    for (auto &n : track_cnt_)
        n++;

    if (1)
    {
        //rejectWithF();  //通过求解F矩阵过程中的RANSAC 来去除outlier
        // 通过setMask去除边缘畸变较大的点
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        //ROS_WARN("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        //光流跟踪不够的用新提取的corners角点来补,使其达到每帧最大的特征点数量
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts_.size());
        if (n_max_cnt > 0)
        {
            if(mask_.empty())
                cout << "mask is empty " << endl;
            if (mask_.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            // 角点提取
            cv::goodFeaturesToTrack(cur_img_, n_pts_, MAX_CNT - cur_pts_.size(), 0.01, MIN_DIST, mask_); 
        }
        else
            n_pts_.clear();
       //ROS_WARN("detect feature costs: %fms", t_t.toc());

        // 把光流没追够的，用corner角点补进去 n_pts + cur_pts == >track_cnt_
        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
       // ROS_WARN("selectFeature costs: %fms", t_a.toc());
    }
    
    //将当前帧的特征点(像素平面 u v)先去畸变，然后映射到归一化坐标系中
    cur_un_pts_ = undistortedPts(cur_pts_, m_camera_[0]);

    //根据前一帧和当前帧的特征点来计算当前帧特征点的速度 vx,vy
    pts_velocity_ = ptsVelocity(ids_, cur_un_pts_, cur_un_pts_map_, prev_un_pts_map_);

    //如果双目的话，根据当前的左图和左图中的特征点，使用光流法来找一下右图中的特征点
    if(!_img1.empty() && stereo_cam_)
    {
        ids_right_.clear();
        cur_right_pts_.clear();
        cur_un_right_pts_.clear();
        right_pts_velocity_.clear();
        cur_un_right_pts_map_.clear();
        if(!cur_pts_.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img_, rightImg, cur_pts_, cur_right_pts_, status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img_, cur_right_pts_, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts_[i]) && distance(cur_pts_[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right_ = ids_;
            reduceVector(cur_right_pts_, status);
            reduceVector(ids_right_, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids_, status);
            reduceVector(track_cnt_, status);
            reduceVector(cur_un_pts_, status);
            reduceVector(pts_velocity_, status);
            */
           //
            cur_un_right_pts_ = undistortedPts(cur_right_pts_, m_camera_[1]);
            right_pts_velocity_ = ptsVelocity(ids_right_, cur_un_right_pts_, cur_un_right_pts_map_, prev_un_right_pts_map_);
        }
        prev_un_right_pts_map_ = cur_un_right_pts_map_;
    }

    // OpenCV 显示特征点与特征追踪的线
    if(SHOW_TRACK)
        drawTrack(cur_img_, rightImg, ids_, cur_pts_, cur_right_pts_, prevLeftPtsMap_);

    // 当前帧赋值到上一帧历史记录
    prev_img_ = cur_img_;
    prev_pts_ = cur_pts_;
    prev_un_pts_ = cur_un_pts_;
    prev_un_pts_map_ = cur_un_pts_map_;
    prev_time_ = cur_time_;
    hasPrediction_ = false;

    prevLeftPtsMap_.clear();
    for(size_t i = 0; i < cur_pts_.size(); i++)
        prevLeftPtsMap_[ids_[i]] = cur_pts_[i];

    //将每一个特征点打包成featureFrame <归一化坐标的x,y,z 像素坐标u,v 点的速度vx,vy>
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids_.size(); i++)
    {
        int feature_id = ids_[i];
        double x, y ,z;
        x = cur_un_pts_[i].x;
        y = cur_un_pts_[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts_[i].x;
        p_v = cur_pts_[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity_[i].x;
        velocity_y = pts_velocity_[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    //打包右图的特征点 (左图camera_id=0 右图camera_id=1)
    if (!_img1.empty() && stereo_cam_)
    {
        for (size_t i = 0; i < ids_right_.size(); i++)
        {
            int feature_id = ids_right_[i];
            double x, y ,z;
            x = cur_un_right_pts_[i].x;
            y = cur_un_right_pts_[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts_[i].x;
            p_v = cur_right_pts_[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity_[i].x;
            velocity_y = right_pts_velocity_[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    ROS_WARN("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

/**
 * @brief  使用F矩阵去除外点
 * @note   
 * @retval None
 */
void FeatureTracker::rejectWithF()
{
    if (cur_pts_.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts_.size()), un_prev_pts(prev_pts_.size());
        for (unsigned int i = 0; i < cur_pts_.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera_[0]->liftProjective(Eigen::Vector2d(cur_pts_[i].x, cur_pts_[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col_ / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row_ / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera_[0]->liftProjective(Eigen::Vector2d(prev_pts_[i].x, prev_pts_[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col_ / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row_ / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts_.size();
        reduceVector(prev_pts_, status);
        reduceVector(cur_pts_, status);
        reduceVector(cur_un_pts_, status);
        reduceVector(ids_, status);
        reduceVector(track_cnt_, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts_.size(), 1.0 * cur_pts_.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}
/**
 * @brief  读取相机模型标定参数
 * @note   
 * @param  &calib_file: 
 * @retval None
 */
void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera_.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam_ = 1;
}
/**
 * @brief  显示去畸变
 * @note   
 * @param  &name: 
 * @retval None
 */
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row_ + 600, col_ + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col_; i++)
        for (int j = 0; j < row_; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera_[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col_ / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row_ / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row_ + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col_ + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img_.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}
/**
 * @brief  无畸变后的点
 * @note   
 * @param  &pts: 
 * @param  cam: 
 * @retval 
 */
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}
/**
 * @brief  特征点速度
 * @note   
 * @param  &ids: 
 * @param  &pts: 
 * @param  map<int: 
 * @param  &cur_id_pts: 
 * @param  &prev_id_pts: 
 * @retval 
 */
vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids_, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity_;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids_.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids_[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time_ - prev_time_;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids_[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity_.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity_.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts_.size(); i++)
        {
            pts_velocity_.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity_;
}

/**
 * @brief  画跟踪点
 * @note   
 * @param  &imLeft: 
 * @param  &imRight: 
 * @param  &curLeftIds: 
 * @param  &curLeftPts: 
 * @param  &curRightPts: 
 * @param  map<int: 
 * @param  &prevLeftPtsMap: 
 * @retval None
 */
void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap_)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam_)
        cv::hconcat(imLeft, imRight, imTrack_);
    else
        imTrack_ = imLeft.clone();
    cv::cvtColor(imTrack_, imTrack_, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt_[j] / 20);
        cv::circle(imTrack_, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam_)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack_, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack_, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap_.find(id);
        if(mapIt != prevLeftPtsMap_.end())
        {
            cv::arrowedLine(imTrack_, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug_.size(); i++)
    {
        cv::circle(imTrack_, predict_pts_debug_[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

/**
 * @brief 设置预测跟踪点
 * @note   
 * @param  &predictPts: 
 * @retval None
 */
void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction_ = true;
    predict_pts_.clear();
    predict_pts_debug_.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids_.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids_[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera_[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts_.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug_.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts_.push_back(prev_pts_[i]);
    }
}
/**
 * @brief  移除外点
 * @note   
 * @param  &removePtsIds: 
 * @retval None
 */
void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids_.size(); i++)
    {
        itSet = removePtsIds.find(ids_[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts_, status);
    reduceVector(ids_, status);
    reduceVector(track_cnt_, status);
}
/**
 * @brief  返回跟踪图像
 * @note   
 * @retval 
 */
cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack_;
}

FeatureTracker::~FeatureTracker()
{
}
