#pragma once 

#include <vector>
#include "../parameters.h"
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <ros/console.h>

// ！！！重要，主函数标定外参旋转矩阵
/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
class InitialEXRotation
{
public:
	InitialEXRotation();

    // ！！！重要，主函数标定外参旋转矩阵
    bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
private:
    // 两帧之间相机系旋转矩阵 
	Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    // 本质矩阵SVD分解求得4组RT 
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
    int frame_count;

    vector< Matrix3d > Rc;// 相机之间旋转矩阵，对极几何得到
    vector< Matrix3d > Rimu;// 两个IMU之间旋转矩阵，由IMU预积分得到
    vector< Matrix3d > Rc_g;// 每个IMU相对于起始IMU的旋转矩阵
    Matrix3d ric;// 相机到IMU的外参
};


