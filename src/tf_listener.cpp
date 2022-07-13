#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "listener");
    
    ros::NodeHandle node;
    ros::service::waitForService("spawn");
    ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_turtle;
    spawn_turtle.request.name = "turtle2";
    spawn_turtle.request.x = 4.0;
    spawn_turtle.request.y = 2.0;
    spawn_turtle.request.theta = 0.0;
    spawner.call(spawn_turtle);

    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate rate(10);
    while (node.ok()){
        geometry_msgs::TransformStamped stamp;
        try {
            stamp = buffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
        }
        catch (tf2::TransformException &e){
            ROS_WARN("%s", e.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist twist;
        twist.linear.x = 1.0 / sqrt(pow(stamp.transform.translation.x,2) + pow(stamp.transform.translation.y, 2));
        twist.angular.z = - 0.2 * atan2(stamp.transform.translation.y, stamp.transform.translation.x);

        publisher.publish(twist);

        rate.sleep();
    }
    
    return 0;
}