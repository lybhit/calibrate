#ifndef MATCHER_H
#define MATCHER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace calibrate{

using Point3d = Eigen::Vector3d;

 class Matcher
 {
 public:
     virtual void match(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2) = 0;
     virtual Eigen::Matrix3d getRot() = 0;
     virtual Eigen::Vector3d getTrans()  = 0;
 };

} //namespace calibrate

#endif
