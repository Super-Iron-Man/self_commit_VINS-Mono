#include "initial_ex_rotation.h"
/* 标定IMU和相机的外参数 */
InitialEXRotation::InitialEXRotation(){
    frame_count = 0;
    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(Matrix3d::Identity());
    ric = Matrix3d::Identity();
}
/*
输入参数为：vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, 匹配的特征点 和 IMU预积分得的旋转矩阵Q
输出参数：Matrix3d &calib_ric_result    标定的外参数
*/
bool InitialEXRotation::CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{
    frame_count++;
    Rc.push_back(solveRelativeR(corres));// 相机帧之间旋转矩阵
    Rimu.push_back(delta_q_imu.toRotationMatrix());//IMU之间旋转矩阵，由IMU预积分得到
    Rc_g.push_back(ric.inverse() * delta_q_imu * ric);//每帧IMU相对于起始帧IMU的R，初始化为IMU预积分

    // 2.SVD分解，Ax=0,对A填充
    Eigen::MatrixXd A(frame_count * 4, 4);// 多帧组成A,一对点组成的A是4*4的。
    A.setZero();
    int sum_ok = 0;
    for (int i = 1; i <= frame_count; i++)
    {
         // 2.1求解核函数
        Quaterniond r1(Rc[i]);
        Quaterniond r2(Rc_g[i]);

        double angular_distance = 180 / M_PI * r1.angularDistance(r2);//算两个坐标系之间相对旋转矩阵在做轴角变换后(u * theta)的角度theta
        //theta越小说明两个坐标系的姿态越接近，这个角度距离用于后面计算权重，这里计算权重就是为了降低外点的干扰，意思就是为了防止出现误差非常大的R_bk+1^bk和 R_ck+1^ck约束导致估计的结果偏差太大 
        ROS_DEBUG(
            "%d %f", i, angular_distance);
        //huber核函数做加权
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
        ++sum_ok;
        // 2.2 分别为L和R赋值，L R 分别为左乘和右乘矩阵，都是4*4矩阵
        //R_bk+1^bk * R_c^b = R_c^b * R_ck+1^ck
        //[Q1(q_bk+1^bk) - Q2(q_ck+1^ck)] * q_c^b = 0
        Matrix4d L, R;

        double w = Quaterniond(Rc[i]).w();// 相机间旋转矩阵,实部
        Vector3d q = Quaterniond(Rc[i]).vec();// 相机间旋转矩阵,虚部
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        Quaterniond R_ij(Rimu[i]);// IMU间旋转矩阵
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;
        // A=Q1(q_bk+1^bk) - Q2(q_ck+1^ck)=huber(L-R)
        A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }
    // 2.3 svd分解中最小奇异值对应的右奇异向量作为旋转四元数
    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    //这里的四元数存储的顺序是[x,y,z,w]',即[qv qw]'
    Matrix<double, 4, 1> x = svd.matrixV().col(3);// 右奇异向量作为旋转四元数
    Quaterniond estimated_R(x);
    ric = estimated_R.toRotationMatrix().inverse();// 求逆
    //cout << svd.singularValues().transpose() << endl;
    //cout << ric << endl;
    Vector3d ric_cov;
    ric_cov = svd.singularValues().tail<3>();
    if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)
    {
        calib_ric_result = ric;
        return true;
    }
    else
        return false;
}

//根据两帧归一化特征点求解两帧的旋转矩阵
Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if (corres.size() >= 9)// 需要特征点大于9对，否则返回单位矩阵
    {
        // 归一化相机系下前二维坐标SVD分解
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        // 求解两帧的本质矩阵E
        cv::Mat E = cv::findFundamentalMat(ll, rr);
        cv::Mat_<double> R1, R2, t1, t2;
        // 本质矩阵svd分解得到4组Rt解
        decomposeE(E, R1, R2, t1, t2);
        // 如果行列式为负，SVD分解-E
        if (determinant(R1) + 1.0 < 1e-09)
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }

        // 通过三角化得到的正深度选择Rt解
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;

        // 对R求转置
        Matrix3d ans_R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j);
        return ans_R_eigen;
    }
    return Matrix3d::Identity();
}

double InitialEXRotation::testTriangulation(const vector<cv::Point2f> &l,
                                          const vector<cv::Point2f> &r,
                                          cv::Mat_<double> R, cv::Mat_<double> t)
{
    cv::Mat pointcloud;
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);
    cv::Matx34f P1 = cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
                                 R(1, 0), R(1, 1), R(1, 2), t(1),
                                 R(2, 0), R(2, 1), R(2, 2), t(2));
    cv::triangulatePoints(P, P1, l, r, pointcloud);// 三角化得到路标3D坐标
    int front_count = 0;
    for (int i = 0; i < pointcloud.cols; i++)
    {
        double normal_factor = pointcloud.col(i).at<float>(3);

        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);
        if (p_3d_l(2) > 0 && p_3d_r(2) > 0)
            front_count++;
    }
    ROS_DEBUG("MotionEstimator: %f", 1.0 * front_count / pointcloud.cols);
    return 1.0 * front_count / pointcloud.cols;
}
//本质矩阵E分解得到R T
void InitialEXRotation::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}
