#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include <fstream>

class OdomHandle{
    public:
    OdomHandle(const std::string topic_name, std::string save_file_name, const std::string path_name);
    ~OdomHandle();

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    void writeDataToFile(const geometry_msgs::PoseStamped& pose);

    // void setOdomTopicName(std::string topic_name);
    // void setSaveFileName(std::string save_file_name);

    private:
    ros::NodeHandle nh_;
    // ros::NodeHandle private_nh_;

    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;

    std::string topic_name_;
    std::string path_name_;
    
    std::ofstream out_file_;
    std::string file_name_;

    bool first_msg_;
    double first_time_stamp_;
};


OdomHandle::OdomHandle(const std::string topic_name, std::string save_file_name, const std::string path_name):
topic_name_(topic_name),
file_name_(save_file_name),
path_name_(path_name),
first_msg_(false)
{
    const char* file = file_name_.c_str();
    out_file_.open(file,std::ios::app);

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(topic_name_, 5, &OdomHandle::odomCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_name_, 100);

}

void OdomHandle::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    static nav_msgs::Path path;
    if(!first_msg_)
    {
        first_time_stamp_ = msg->header.stamp.toSec();
        first_msg_ = true;
    }
    geometry_msgs::PoseStamped pp;
    pp.header = msg->header;
    pp.pose = msg->pose.pose;
    path.poses.push_back(pp);
    path_pub_.publish(path);

    writeDataToFile(pp);
}

void OdomHandle::writeDataToFile(const geometry_msgs::PoseStamped& pose)
{
    if(out_file_.is_open())
    {
        out_file_ << pose.pose.position.x << ' ' << pose.pose.position.y << ' ' << pose.pose.position.z << ' ' << pose.header.stamp.toSec() - first_time_stamp_ << '\n';
    }

}

OdomHandle::~OdomHandle()
{
    if(out_file_.is_open())
    {
        out_file_.close();
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_handle");

    ros::NodeHandle private_nh("~");
    std::string topic_name;
    std::string save_file_name;
    std::string path_name;

    private_nh.param<std::string>("topic_name", topic_name, "odom");
    private_nh.param<std::string>("save_file_name", save_file_name, "file");
    private_nh.param<std::string>("path_name", path_name, "path");

    OdomHandle odom_handle(topic_name, save_file_name, path_name);

    ros::spin();

    return 0;
}


