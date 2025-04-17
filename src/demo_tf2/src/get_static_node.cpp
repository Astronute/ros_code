#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "get_static_node");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate r(1);

    while(ros::ok()){
        // 创建一个在“laser”坐标系下的坐标点
        geometry_msgs::PointStamped point_laser; // 坐标点坐标信息数据类型
        point_laser.header.frame_id = "laser";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 2.0;
        point_laser.point.y = 3.0;
        point_laser.point.z = 5.0;
        
        try{
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser, "base_link");
            ROS_INFO("point from laser->%s RESULT:(%.2f,%.2f,%.2f)",point_base.header.frame_id.c_str(), point_base.point.x, point_base.point.y, point_base.point.z);
        }
        catch(const std::exception& e){
            ROS_INFO("error");
        }
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
