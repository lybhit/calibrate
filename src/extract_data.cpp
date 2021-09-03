#include "extract_data.h"

namespace calibrate{

   ExtractData::ExtractData(){}

    ExtractData::~ExtractData(){}

    void ExtractData::setSourceTopicName(const std::string str)
    {
        odom_topic_0_ = str;
    }

    void ExtractData::setTargetTopicName(const std::string str)
    {
        odom_topic_1_ = str;
    }

    void ExtractData::setInputBagName(const std::string str)
    {
        input_bag_str_ = str;
    }

    void ExtractData::bagRead()
    {
        bag_.open(input_bag_str_, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(odom_topic_0_);
        topics.push_back(odom_topic_1_);
        rosbag::View view(bag_, rosbag::TopicQuery(topics));

        // Sleep for a second to let all subscribers connect
        ros::WallDuration(1.0).sleep();

        // ros::WallTime start(ros::WallTime::now());

        for(auto &it : view)
        {
            // Process any ros messages or callbacks at this point
            // ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());
            if(it.getTopic() == odom_topic_0_) 
            {
                nav_msgs::Odometry::ConstPtr msg_ptr = it.instantiate<nav_msgs::Odometry>();
                points_source_.push_back(poseFromMsg(msg_ptr));
            
            }else if(it.getTopic() == odom_topic_1_){
                nav_msgs::Odometry::ConstPtr msg_ptr = it.instantiate<nav_msgs::Odometry>();
                points_target_.push_back(poseFromMsg(msg_ptr));
            }

        }
        bag_.close();
    }

    Pose3 ExtractData::poseFromMsg(nav_msgs::Odometry::ConstPtr msg)
    {
        Pose3 temp;
        Eigen::Quaterniond q(msg->pose.pose.orientation.w,  msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Vector3d pose = - q.toRotationMatrix().transpose() * t;

        temp.x() = pose.x();
        temp.y() = pose.y();
        temp.z() = pose.z();
        temp.w() = msg->header.stamp.toSec();
        return temp;
    }

    void  ExtractData::findNearestPairs()
    {
        std::cout << "points_source size = " << points_source_.size() << std::endl;
        std::cout << "points_target size = " << points_target_.size() << std::endl;

        auto it_source = points_source_.begin();
        auto it_target = points_target_.begin();
        auto it_target_select = points_target_.begin();


        if(points_source_.begin()->w() > std::prev(points_target_.end())->w() || std::prev(points_source_.end())->w() < points_target_.begin()->w())
        {
            std::cout << "two sets points time has a large gap." << std::endl;
            return;
        }

        for(; it_source != points_source_.end(); it_source++)
        {
            double time = it_source->w();
            for(; it_target != points_target_.end(); it_target++)
            {
                if(it_target->w() >= time)
                {
                    auto pre = std::prev(it_target);
                    if(std::abs(pre->w() - time) < std::abs(it_target->w() - time))
                    {
                        it_target_select = pre;
                    }else{
                        it_target_select = it_target;
                    }
                    break;
                }

            }
            points_pair_.emplace_back(std::pair<Pose3, Pose3>(*it_source, *it_target_select));
        } 

    }

    void ExtractData::saveDataToTxT(const std::string str)
    {
        const char* file = str.c_str();
        out_file_.open(file, std::ios::app);

        out_file_ << "pose base_link and imu in world" << '\n';

        double ref_time = points_pair_.begin()->first.w();

        for(auto &elem : points_pair_)
        {
            out_file_ << elem.first.x() << ' ' << elem.first.y() << ' ' << elem.first.z() << ' ' << elem.first.w()  - ref_time<< ', ' 
                                << elem.second.x() << ' '<< elem.second.y() << ' ' << elem.second.z() << ' ' << elem.second.w() - ref_time << '\n';
        }

        out_file_.close();
    }

    const std::vector<std::pair<Pose3, Pose3>> &   ExtractData::getPointsPare()
    {
        return points_pair_;
    }
}