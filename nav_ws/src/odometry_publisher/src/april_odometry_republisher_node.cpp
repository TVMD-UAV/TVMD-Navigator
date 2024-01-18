#include <cstdio>
#include <unistd.h>
#include <cstdlib>
#include <cstring>

#include <ros/ros.h>
// #include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

class OdometryRepublisher {
public:
    static std::string odometry_base_frame;
    static std::string fcu_frame;

    OdometryRepublisher(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("mavros/odometry/out", 10);
        odom_sub_ = nh_.subscribe("tag_detections", 10, &OdometryRepublisher::odom_cb);
        
        nh_.param<std::string>("odometry_base_frame", odometry_base_frame, "map");
        nh_.param<std::string>("fcu_frame", fcu_frame, "base_link");
        ROS_INFO_STREAM("Odometry republisher initiated: " << odometry_base_frame << "=>" << fcu_frame);
    }

private:
    static void odom_cb(const apriltag_ros::AprilTagDetectionArrayPtr & tag_in) 
    {
        nav_msgs::Odometry odom_out;
        odom_out.header = tag_in->header;
        odom_out.header.frame_id = "marker_frame";         // Coordinates of pose frame
                                                    // A transformation from odom to world must be provided to complete the tf tree
        odom_out.child_frame_id = "camera_color_optical_frame";  // Coordinates of twist

        if (tag_in->detections.size() > 0) {
            // The detection results are expressed in camera_optical_frame
            tf2::Transform transform;
            // tag_in->detections[0].pose.pose.pose is geometry_msgs::Pose 
            tf2::fromMsg(tag_in->detections[0].pose.pose.pose, transform);

            // odom_out.pose.pose is geometry_msgs::Pose 
            tf2::toMsg(transform.inverse(), odom_out.pose.pose);

            // set body frame manually
            odom_out.twist.twist.linear.x = 0;
            odom_out.twist.twist.linear.y = 0;
            odom_out.twist.twist.linear.z = 0;
            odom_pub_.publish(odom_out);
        }
    }

    // static inline void pose_msg_to_tf(const geometry_msgs::Pose& msg, tf::Transform& bt)
    // {
    //     bt = tf::Transform(
    //         tf::Quaternion(msg.orientation.x, 
    //             msg.orientation.y, 
    //             msg.orientation.z, 
    //             msg.orientation.w ), 
    //         tf::Vector3(msg.position.x, 
    //             msg.position.y, 
    //             msg.position.z));
    // }

    ros::NodeHandle nh_;
    static ros::Publisher  odom_pub_;
    static ros::Subscriber odom_sub_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "april_odometry_republisher_node");
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
