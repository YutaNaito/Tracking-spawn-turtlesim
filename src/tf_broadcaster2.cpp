#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>

void callback(const turtlesim::PoseConstPtr pose){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped stamp;

    stamp.header.stamp = ros::Time(0);
    stamp.header.frame_id = "world";
    stamp.child_frame_id = "turtle2";
    stamp.transform.translation.x = pose->x;
    stamp.transform.translation.y = pose->y;
    stamp.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose->theta);
    stamp.transform.rotation.x = q.x();
    stamp.transform.rotation.y = q.y();
    stamp.transform.rotation.z = q.z();
    stamp.transform.rotation.w = q.w();

    br.sendTransform(stamp);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "t_broadcaster2");
    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe("/turtle2/pose", 10, &callback);
    ros::spin();

    return 0;
}