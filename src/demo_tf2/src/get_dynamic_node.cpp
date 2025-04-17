#include <ros/ros.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


// 计算turtle坐标系中的点在world坐标系中的坐标，turtle坐标系移动
// 使用tf2_ros::Buffer对象来进行转换
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "get_dynamic_node");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while(ros::ok()){
        // 生成基于turtle坐标系的点（雷达点云坐标）
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "turtle1";
        point_laser.header.stamp = ros::Time();
        point_laser.point.x = 1;
        point_laser.point.y = 1;
        point_laser.point.z = 0;

        try{
            // 将turtle坐标系中的点转换到world坐标系下
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser, "world");
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
