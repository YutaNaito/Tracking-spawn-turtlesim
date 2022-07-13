#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

void callback(const turtlesim::PoseConstPtr& pose){
    if (pose->x <= 0 || pose->x >= 11 || pose->y <= 0 || pose->y >= 11 ){
        geometry_msgs::Twist twist;
        twist.linear.x = 0.8;
        twist.angular.z = 1.0;

        ros::NodeHandle node;
        ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
        publisher.publish(twist);        
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "subscribe_turtle");

    ros::NodeHandle node;
    ros::Subscriber subscriber = node.subscribe("/turtle2/pose", 10, &callback);
    ros::spin();

    return 0;
};
