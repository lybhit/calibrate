#ifndef ICP_PCL_MATCHER_H
#define ICP_PCL_MATCHER_H

#include "matcher.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace calibrate{

    typedef pcl::PointXYZ pointT; 

    class IcpMatcher : public Matcher 
    {
        public:

            virtual void match(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2) override;

            void dataToPclFormat(const std::vector<Point3d> &pts_1, const std::vector<Point3d> &pts_2);

            virtual Eigen::Matrix3d getRot() override;
            virtual Eigen::Vector3d getTrans() override;

        private:
            Eigen::Matrix3d rotate_matrix_;
            Eigen::Vector3d translation_vec_;

            pcl::PointCloud<pointT>::Ptr source_;
            pcl::PointCloud<pointT>::Ptr target_;

    };

}// namespace calibrate

typedef pcl::PointXYZ PointType;


#endif
