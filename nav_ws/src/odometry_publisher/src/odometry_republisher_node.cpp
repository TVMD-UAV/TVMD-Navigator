#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <cstring>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class OdometryRepublisher {
public:
    static std::string odometry_base_frame;
    static std::string fcu_frame;

    OdometryRepublisher(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("mavros/odometry/out", 10);
        odom_sub_ = nh_.subscribe("vins_node/odometry", 10, &OdometryRepublisher::odom_cb);
        // odom_pub_ = nh_.advertise<nav_msgs::Odometry>("mavros/vision_pose/pose", 10);
        
        nh_.param<std::string>("odometry_base_frame", odometry_base_frame, "map");
        nh_.param<std::string>("fcu_frame", fcu_frame, "base_link");
        ROS_INFO_STREAM("Odometry republisher initiated: " << odometry_base_frame << "=>" << fcu_frame);
    }

private:
    static void odom_cb(const nav_msgs::OdometryPtr & odom_in) 
    {
        nav_msgs::Odometry odom_out = *odom_in;
        odom_out.header.frame_id = "world";         // Coordinates of pose frame
                                                    // A transformation from odom to world must be provided to complete the tf tree
        odom_out.child_frame_id = "base_link_frd";  // Coordinates of twist

        // VINS publish twist in world frame, so we need to transform it to
        // body frame manually
        odom_out.twist.twist.linear.x = -odom_out.twist.twist.linear.y;
        odom_out.twist.twist.linear.y =  odom_out.twist.twist.linear.z;
        odom_out.twist.twist.linear.z = -odom_out.twist.twist.linear.x;
        odom_pub_.publish(odom_out);
    }

    ros::NodeHandle nh_;
    static ros::Publisher  odom_pub_;
    static ros::Subscriber odom_sub_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "odometry_republisher_node");
    ros::NodeHandle nh;
    OdometryRepublisher republisher(&nh);
    
    ros::spin();
    ros::shutdown();
    
    return 0;
}

ros::Publisher  OdometryRepublisher::odom_pub_;
ros::Subscriber OdometryRepublisher::odom_sub_;

std::string OdometryRepublisher::odometry_base_frame;
std::string OdometryRepublisher::fcu_frame;
