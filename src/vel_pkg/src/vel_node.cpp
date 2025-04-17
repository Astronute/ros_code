#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h> // action msg

ros::Publisher vel_pub;

bool stop = true;

void ImuCallback(const sensor_msgs::Imu msg){
    if(msg.orientation_covariance[0] < 0){
        return ;
    }
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    roll = roll * 180/M_PI;
    pitch = pitch * 180/M_PI;
    yaw = yaw * 180/M_PI;
    ROS_INFO("roll:%f pitch:%f yaw:%f", roll, pitch, yaw);

    geometry_msgs::Twist ctrl_msg;
    double target_yaw = 90;
    double err_yaw = target_yaw - yaw;
    ctrl_msg.angular.z = err_yaw*0.01;
    vel_pub.publish(ctrl_msg);
}

void LidarCallback(const sensor_msgs::LaserScan msg){
    geometry_msgs::Twist vel_msg;
    float distanceFront = msg.ranges[180];
    ROS_INFO("distance front:%f", distanceFront);
    if(stop){
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;
        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = 0.0;
    }

    vel_pub.publish(vel_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "velo_node");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &LidarCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, &ImuCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::spin();

    return 0;
}
