#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

bool inORBBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 31;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
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
}

void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        // if image is too dark or light, trun on equalize to find enough features
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i])) 
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++; 

    if (PUB_THIS_FRAME)
    {
        rejectWithF(); 
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask(); 
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
            vector<uchar> statusn(n_pts.size(), 1);
            for (int i = 0; i < int(n_pts.size()); i++)
                if (!inBorder(n_pts[i]))  // Exclude points that are too close to the image boundary
                    statusn[i] = 0;
            reduceVector(n_pts, statusn);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints(); 
    prev_time_reserved = prev_time; 
    cur_time_reserved = cur_time;
    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b); 
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z())); 
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1) 
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0)); 
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

// void addOffsetToKeypoints(std::vector<cv::KeyPoint>& keypoints, int offsetX, int offsetY) {
//     for (cv::KeyPoint& kp : keypoints) {
//         kp.pt.x += offsetX;
//         kp.pt.y += offsetY;
//     }
// }

// void subtractOffsetToKeypoints(std::vector<cv::KeyPoint>& keypoints, int offsetX, int offsetY) {
//     for (cv::KeyPoint& kp : keypoints) {
//         kp.pt.x -= offsetX;
//         kp.pt.y -= offsetY;
//     }
// }

static inline float getScale(int level, int firstLevel, double scaleFactor)
{
    return (float)std::pow(scaleFactor, (double)(level - firstLevel));
}

static void ICAngles(const cv::Mat& img, const std::vector<cv::Rect>& layerinfo,
                     std::vector<cv::KeyPoint>& pts, const std::vector<int> & u_max, int half_k)
{
    int step = (int)img.step1(); 
    size_t ptidx, ptsize = pts.size(); 

    for( ptidx = 0; ptidx < ptsize; ptidx++ ) 
    {
        const cv::Rect& layer = layerinfo[pts[ptidx].octave]; 
        const uchar* center = &img.at<uchar>(cvRound(pts[ptidx].pt.y) + layer.y, cvRound(pts[ptidx].pt.x) + layer.x);
        int m_01 = 0, m_10 = 0; 

        // Treat the center line differently, v=0
        for (int u = -half_k; u <= half_k; ++u)
            m_10 += u * center[u]; 

        // Go line by line in the circular patch
        for (int v = 1; v <= half_k; ++v)
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v]; 
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v*step], val_minus = center[u - v*step]; 
                v_sum += (val_plus - val_minus); 
                m_10 += u * (val_plus + val_minus); 
            }
            m_01 += v * v_sum; 
        }

        pts[ptidx].angle = cv::fastAtan2((float)m_01, (float)m_10); 
    }
}

void FeatureTracker::descriptorsGet()
{
    cur_pts_descriptors.clear();
    float scale = 1.0;

    ROS_INFO("initial cur_pts.size(): %d", (int)cur_pts.size());
    std::vector<cv::KeyPoint> keypoints;
    for (size_t i = 0; i < cur_pts.size(); i++)
    {
        keypoints.push_back(cv::KeyPoint(cur_pts[i], scale, -1, 0, 0, -1));
    }

    /*-- Get uamx */
    std::vector<int> umax(halfPatchSize + 2);
    int v, v0, vmax = cvFloor(halfPatchSize * std::sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(halfPatchSize * std::sqrt(2.f) / 2);
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(std::sqrt((double)halfPatchSize * halfPatchSize - v * v));

    // Make sure we are symmetric
    for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }

    /*-- Get layerInfo */
    std::vector<cv::Rect> layerInfo(1);
    int level_dy = ROW + border * 2;
    cv::Point level_ofs(0, 0);
    cv::Size bufSize((cvRound(COL / getScale(0, firstLevel, scaleFactor)) + border * 2 + 15) & -16, 0);
    cv::Size sz(cvRound(COL / scale), cvRound(ROW / scale));
    cv::Size wholeSize(sz.width + border * 2, sz.height + border * 2);
    if (level_ofs.x + wholeSize.width > bufSize.width)
    {
        level_ofs = cv::Point(0, level_ofs.y + level_dy);
        level_dy = wholeSize.height;
    }

    cv::Rect linfo(level_ofs.x + border, level_ofs.y + border, sz.width, sz.height);
    layerInfo[0] = linfo;
    level_ofs.x += wholeSize.width;
    bufSize.height = level_ofs.y + level_dy;

    /*-- Get imagePyramid */
    cv::Mat imagePyramid;
    imagePyramid.create(bufSize, CV_8U);
    // Pre-compute the scale pyramids
    cv::Rect linform = layerInfo[0];
    cv::Size sze(linform.width, linform.height);
    cv::Size wholeSz(sze.width + border * 2, sze.height + border * 2);
    cv::Rect wholeLinfo = cv::Rect(linform.x - border, linform.y - border, wholeSz.width, wholeSz.height);
    cv::Mat extImg = imagePyramid(wholeLinfo);
    copyMakeBorder(cur_img, extImg, border, border, border, border,
                   cv::BORDER_REFLECT_101);

    /*-- Get Angles */
    ICAngles(imagePyramid, layerInfo, keypoints, umax, halfPatchSize);

    std::vector<cv::KeyPoint> originalKeypoints = keypoints;
    cv::Ptr<cv::DescriptorExtractor> orb_descriptor = cv::ORB::create();
    cv::Mat descriptors;
    orb_descriptor->compute(cur_img, keypoints, descriptors);

    std::vector<int> indices_without_descriptor;
    size_t i = 0; 
    size_t j = 0; 
    while (i < originalKeypoints.size() && j < keypoints.size())
    {
        if (originalKeypoints[i].pt == keypoints[j].pt)
        {
            ++i;
            ++j;
        }
        else
        {
            indices_without_descriptor.push_back(i);
            ++i;
        }
    }
    while (i < originalKeypoints.size()) 
    {
        indices_without_descriptor.push_back(i);
        ++i;
    }

    /*  */
    cv::Mat Descriptor128(1, descriptors.cols, CV_8UC1, cv::Scalar(128));
    for (auto it = indices_without_descriptor.begin(); it != indices_without_descriptor.end(); ++it) 
    {
        int index = *it;
        cv::Mat upperPart = descriptors.rowRange(0, index);
        cv::Mat lowerPart = descriptors.rowRange(index, descriptors.rows);
        if (upperPart.empty())
        {
            upperPart = Descriptor128;
        }
        else
        {
            cv::vconcat(upperPart, Descriptor128, upperPart);
        }
        if (lowerPart.empty())
        {
            descriptors = upperPart;
        }
        else
        {
            cv::vconcat(upperPart, lowerPart, descriptors);
        }
    }

    ROS_INFO("descriptors.rows: %d", descriptors.rows);
    for (int i = 0; i < descriptors.rows; i++)
    {
        cur_pts_descriptors[ids[i]] = descriptors.row(i);
        cur_pts_ids[ids[i]] = cur_pts[i];
    }
    ROS_INFO("described cur_pts.size(): %d", (int)cur_pts.size());
}

void FeatureTracker::CLGet()
{
    int old_pts_num = 0;
    multi_track_cur_pts_track_num = 1;
    for (int i = 0; i < (int)track_cnt.size(); i++)
    {
        if (track_cnt[i] > 1)
            old_pts_num++;

        if (track_cnt[i] > 11)
            multi_track_cur_pts_track_num = multi_track_cur_pts_track_num + 1;
    }

    cout << "multi_track_cur_pts_track_num: " << multi_track_cur_pts_track_num << endl;
    cout << "old_pts_num: " << old_pts_num << endl;

    cur_pts_CL.clear();
    std::unordered_map<int, double> debut_pts_cumulate_pixelDistance_tmp;
    std::vector<int> indices_to_remove;
    std::vector<double> indices_to_remove_pixelDistance;
    std::vector<double> Hamming_distances;
    double track_regression = 1.5;

    for (auto CPDIter = cur_pts_descriptors.begin(); CPDIter != cur_pts_descriptors.end(); ++CPDIter)
    {
        if (prew_pts_descriptors.find(CPDIter->first) != prew_pts_descriptors.end())
        {
            /* Get Hamming distance */
            cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
            vector<cv::DMatch> matche;
            matcher->match(prew_pts_descriptors.at(CPDIter->first), cur_pts_descriptors.at(CPDIter->first), matche);
            Hamming_distances.push_back(matche[0].distance);
        }
    }

    int hamming_index = 0;
    for (auto CPDIter = cur_pts_descriptors.begin(); CPDIter != cur_pts_descriptors.end(); ++CPDIter)
    {
        double CLDescriptor = 0;
        double pixelDistance;
        if (prew_pts_descriptors.find(CPDIter->first) != prew_pts_descriptors.end())
        {
            auto it = std::find(ids.begin(), ids.end(), CPDIter->first);
            int index = std::distance(ids.begin(), it);

            double HammingDistance = Hamming_distances[hamming_index];
            hamming_index++;

            if ((prew_pts_descriptors.at(CPDIter->first).at<uchar>(0, 0) == 128 && prew_pts_descriptors.at(CPDIter->first).at<uchar>(0, 1) == 128 && prew_pts_descriptors.at(CPDIter->first).at<uchar>(0, 2) == 128 && prew_pts_descriptors.at(CPDIter->first).at<uchar>(0, 3) == 128) ||
                (cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 0) == 128 && cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 1) == 128 && cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 2) == 128 && cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 3) == 128) || 
                (COL == 512 && cur_pts_ids.at(CPDIter->first).x < 180) || (COL == 512 && cur_pts_ids.at(CPDIter->first).x > 332) || (ROW == 512 && cur_pts_ids.at(CPDIter->first).y < 180) || (ROW == 512 && cur_pts_ids.at(CPDIter->first).y > 332) ||
                multi_track_cur_pts_track_num < 0 || solver_sign == INITIAL)
            {
                pixelDistance = 0.0;
            }
            else
            {
                pixelDistance = max(0.6093 * exp(0.0224 * HammingDistance) - 0.6093 - 1.0, 0.0);
                // pixelDistance = max(0.9858 * exp(0.0220 * HammingDistance) - 0.9858 - 1.0, 0.0);
                
            }
            double cumulate_pixelDistance = debut_pts_cumulate_pixelDistance.at(CPDIter->first) + pixelDistance;
            CLDescriptor = 1.5 / cumulate_pixelDistance;

            /* outlier index */
            if (CLDescriptor < 1.5 / 3.0)
            {
                CLDescriptor = 0.0;
                indices_to_remove.push_back(index);
                indices_to_remove_pixelDistance.push_back(pixelDistance);
            }

            debut_pts_cumulate_pixelDistance_tmp[CPDIter->first] = cumulate_pixelDistance;
        }
        else
        {
            if ((cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 0) == 128 && cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 1) == 128 && cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 2) == 128 && cur_pts_descriptors.at(CPDIter->first).at<uchar>(0, 3) == 128) ||
            (COL == 512 && cur_pts_ids.at(CPDIter->first).x < 180) || (COL == 512 && cur_pts_ids.at(CPDIter->first).x > 332) || (ROW == 512 && cur_pts_ids.at(CPDIter->first).y < 180) || (ROW == 512 && cur_pts_ids.at(CPDIter->first).y > 332))
            {
                pixelDistance = 0.75; 
            }
            else
            {
                pixelDistance = 0.5; 
            }
            debut_pts_cumulate_pixelDistance_tmp[CPDIter->first] = pixelDistance;
            CLDescriptor = track_regression / pixelDistance;
        }
        cur_pts_CL[CPDIter->first] = CLDescriptor;
    }
    prew_pts_descriptors = cur_pts_descriptors;
    debut_pts_cumulate_pixelDistance = debut_pts_cumulate_pixelDistance_tmp;

    /* Remove outliers */
    cur_pts_erase.clear();
    std::sort(indices_to_remove.begin(), indices_to_remove.end());
    for (auto it = indices_to_remove.rbegin(); it != indices_to_remove.rend(); ++it)
    {
        int index = *it;

        cur_pts_erase.push_back(cur_pts[index]);

        ids.erase(ids.begin() + index);
        prev_pts.erase(prev_pts.begin() + index);
        cur_pts.erase(cur_pts.begin() + index);
        cur_un_pts.erase(cur_un_pts.begin() + index);
        track_cnt.erase(track_cnt.begin() + index);
        pts_velocity.erase(pts_velocity.begin() + index);
    }
    ROS_INFO("cur_pts.size(): %d", (int)cur_pts.size());
}