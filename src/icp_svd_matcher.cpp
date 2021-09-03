#include "icp_svd_matcher.h"
#include <iostream>

namespace calibrate{

void IcpSVDMatcher::match(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2)
{
    Point3d p_1, p_2;
    int N = pts_1.size();
    for (int i = 0; i < N; ++i) {
        p_1 += pts_1[i];
        p_2 += pts_2[i];
    }
    p_1 /= N;
    p_2 /= N;
//    去质心坐标
    std::vector<Point3d> q_1(N), q_2(N);
    for (int j = 0; j < N; ++j) {
        q_1[j] = pts_1[j] - p_1;
        q_2[j] = pts_2[j] - p_2;
    }
//    计算W=q_1*q_2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; ++i) {
        Eigen::Vector3d a(q_1[i](0), q_1[i](1), q_1[i](2));
        Eigen::Vector3d b(q_2[i](0), q_2[i](1), q_2[i](2));
        W += a * b.transpose();
        
        // W += Eigen::Vector3d(q_1[i].x, q_1[i].y, q_1[i].z) *
        //      Eigen::Vector3d(q_2[i].x, q_2[i].y, q_2[i].z).transpose();
    }
    std::cout << "W=\n" << W << std::endl;
//    对W进行SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    std::cout << "U=\n" << U << std::endl;
    std::cout << "V=\n" << V << std::endl;
//    求R和t
    rotate_matrix_ = U * (V.transpose());
    if(rotate_matrix_.determinant() < 0)
    {
        rotate_matrix_ = -rotate_matrix_;
    }

    Eigen::Vector3d tmp_1(p_1(0), p_1(1), p_1(2));
    Eigen::Vector3d tmp_2(p_2(0), p_2(1), p_2(2));

    translation_vec_ = tmp_1 - rotate_matrix_ * tmp_2;

} 

Eigen::Matrix3d  IcpSVDMatcher::getRot()
{
    return rotate_matrix_;
}

Eigen::Vector3d  IcpSVDMatcher::getTrans()
{
    return translation_vec_;
}

double   IcpSVDMatcher::calculateError(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2)
{
    assert(pts_1.size() == pts_2.size());

    int len = pts_1.size();
    double err = 0.f;
    for(int i = 0; i < len; i++)
    {
        double tmp = (rotate_matrix_ * pts_2[i] + translation_vec_ - pts_1[i]).norm();
        err += tmp;
    }

    std::cout << "whole error = " << err << std::endl;

    return err;
}

} // namespace calibrate