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

#include "parameters.h"
#include "tic_toc.h"
#include <numeric>

using namespace std;
using namespace camodocal;
using namespace Eigen;

enum SolverSign
{
  INITIAL,
  NON_LINEAR
};

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    void descriptorsGet();
    void CLGet();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids; 
    vector<int> track_cnt; 
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;  
    double cur_time;
    double prev_time;

    static int n_id;

    int firstLevel = 0;
    double scaleFactor = 1.2;
    int patchSize = 31;
    int halfPatchSize = patchSize / 2;
    int edgeThreshold = 31;
    const int HARRIS_BLOCK_SIZE = 9;
    int descPatchSize = cvCeil(halfPatchSize * sqrt(2.0));
    int border = std::max(edgeThreshold, std::max(descPatchSize, HARRIS_BLOCK_SIZE / 2)) + 1;

    std::unordered_map<int, cv::Mat> cur_pts_descriptors;
    std::unordered_map<int, cv::Point2f> cur_pts_ids;
    std::unordered_map<int, cv::Mat> prew_pts_descriptors;
    std::unordered_map<int, double> debut_pts_cumulate_pixelDistance;
    std::unordered_map<int, int> debut_pts_cumulate_direction;
    std::unordered_map<int, double> cur_pts_CL;

    double cur_time_reserved; 
    double prev_time_reserved;

    vector<cv::Point2f> cur_pts_erase;

    int multi_track_cur_pts_track_num = 1;

    SolverSign solver_sign = INITIAL; 
};