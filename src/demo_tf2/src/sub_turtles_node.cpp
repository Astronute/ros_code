#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sub_turtles_node");

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle222/cmd_vel", 1000);

    ros::Rate rate(10);
    while(ros::ok()){
        try{
            // turtle1相对turtle2的坐标信息
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("turtle222", "turtle1", ros::Time(0));
            geometry_msgs::Twist twist;
            twist.linear.x = 0.5*sqrt(pow(tfs.transform.translation.x, 2)+pow(tfs.transform.translation.y,2));
            twist.angular.z = 4*atan2(tfs.transform.translation.y, tfs.transform.translation.x);
            pub.publish(twist);
        }
        catch(const std::exception& e){
            ROS_INFO("error");
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
