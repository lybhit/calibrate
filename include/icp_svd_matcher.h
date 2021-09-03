#ifndef ICP_SVD_MATCHER_H
#define ICP_SVD_MATCHER_H

#include "matcher.h"

namespace calibrate{

 class IcpSVDMatcher : public Matcher
 {
 public:
     virtual void match(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2) override;

     double calculateError(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2);

     virtual Eigen::Matrix3d getRot() override;
     virtual Eigen::Vector3d getTrans() override;

private:
     Eigen::Matrix3d rotate_matrix_;
     Eigen::Vector3d translation_vec_;
 };

}

#endif
