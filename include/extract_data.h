#ifndef EXTRACT_DATA_H
#define EXTRACT_DATA_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <fstream>

#include <Eigen/Dense>

namespace calibrate{

    using Pose3 = Eigen::Vector4d;

    class ExtractData 
    {
    public:
        ExtractData(/* args */);
        ~ExtractData();

        void bagRead();
        void findNearestPairs();

        void setSourceTopicName(const std::string str);
        void setTargetTopicName(const std::string str);
        void setInputBagName(const std::string str);

        void saveDataToTxT(const std::string str);

        const std::vector<std::pair<Pose3, Pose3>> & getPointsPare();

    private:

        Pose3 poseFromMsg(nav_msgs::Odometry::ConstPtr msg);

        rosbag::Bag bag_;
        std::string odom_topic_0_;
        std::string odom_topic_1_;
        std::string input_bag_str_;

        std::vector<Pose3> points_source_;
        std::vector<Pose3> points_target_;
        std::vector<std::pair<Pose3, Pose3>> points_pair_;

        std::ofstream out_file_;
        std::string file_name_;

    };


} // calibrate


#endif