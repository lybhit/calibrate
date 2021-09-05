#include "ros/ros.h"
#include "extract_data.h"
#include "icp_svd_matcher.h"
#include "icp_pcl_matcher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibrate_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string source_topic, target_topic;
    std::string bag_name;
    std::string save_file_name;

    private_nh.param<std::string>("source_topic", source_topic, "odom_0");
    private_nh.param<std::string>("target_topic", target_topic, "odom_1");
    private_nh.param<std::string>("bag_name", bag_name, "input.bag");
    private_nh.param<std::string>("save_file_name", save_file_name , "file_name");

    bool use_debug;
    private_nh.param<bool>("use_debug", use_debug, false);

    calibrate::ExtractData extract_data;
    extract_data.setSourceTopicName(source_topic); 
    extract_data.setTargetTopicName(target_topic);
    extract_data.setInputBagName(bag_name);

    extract_data.bagRead();
    extract_data.findNearestPairs();
    if(use_debug)
    {
        extract_data.saveDataToTxT(save_file_name);
    }

    const std::vector<std::pair<calibrate::Pose3,calibrate::Pose3 >>& pp = extract_data.getPointsPare();

    if(pp.empty())
    {
        std::cout << "points pair is empty, exit." << std::endl;
        return -1;
    }

    std::vector<calibrate::Point3d> pts_1;
    std::vector<calibrate::Point3d> pts_2;
    for(int i = 0; i < (int)pp.size(); i++)
    {
        pts_2.emplace_back(calibrate::Point3d(pp[i].first.x(), pp[i].first.y(), pp[i].first.z()));
        pts_1.emplace_back(calibrate::Point3d(pp[i].second.x(), pp[i].second.y(), pp[i].second.z()));
    }

    assert(pts_1.size() >0);
    assert(pts_1.size() == pts_2.size());

    std::shared_ptr<calibrate::Matcher> match_ptr = std::make_shared<calibrate::IcpMatcher>();
    match_ptr->match(pts_1, pts_2);
    Eigen::Matrix3d R = match_ptr->getRot();
    Eigen::Vector3d t = match_ptr->getTrans();

    std::cout << "R = " << R << std::endl;
    std::cout << "t = " << t << std::endl;

    return 0;
}