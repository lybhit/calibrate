#include "icp_pcl_matcher.h"
// #include <memory>

namespace calibrate{

void IcpMatcher::match(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2)
{
    dataToPclFormat(pts_1, pts_2);

    static pcl::IterativeClosestPoint<pointT, pointT> icp;
    icp.setUseReciprocalCorrespondences(true);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(source_);
    icp.setInputTarget(target_);
    pcl::PointCloud<pointT>::Ptr unused_result(new pcl::PointCloud<pointT>());
    icp.align(*unused_result);

    // ROS_INFO("ICP fitnessScore = %f", icp.getFitnessScore());

    if (icp.hasConverged() == false)
    {
        return;
    }
        
    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    // transform from world origin to corrected pose
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);

    rotate_matrix_ = correctionLidarFrame.matrix().block<3,3>(0, 0).cast<double>();
    translation_vec_ = correctionLidarFrame.matrix().block<3,1>(0, 3).cast<double>();

    // ROS_INFO("ICP laser scan to map result = %f, %f, %f, %f, %f, %f", x, y, z, roll, pitch, yaw);
    float noiseScore = icp.getFitnessScore();
    std::cout << "pcl icp fitness = " << noiseScore << std::endl;

}

void IcpMatcher::dataToPclFormat(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2)
{
    assert(pts_1.size() == pts_2.size());

    source_.reset(new pcl::PointCloud<pointT>());
    target_.reset(new pcl::PointCloud<pointT>());

    int len = pts_1.size();
    
    for(int i = 0; i < len; i++)
    {
        source_->push_back(pointT(pts_2[i].x(), pts_2[i].y(), pts_2[i].z()));
        target_->push_back(pointT(pts_1[i].x(), pts_1[i].y(), pts_1[i].z()));
    }

}

Eigen::Matrix3d  IcpMatcher::getRot()
{
    return rotate_matrix_;
}

Eigen::Vector3d  IcpMatcher::getTrans()
{
    return translation_vec_;
}

}// namespace calibrate