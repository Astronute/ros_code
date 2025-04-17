#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// 发布静态坐标
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "demo_tf2_node");
    tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped ts; // 坐标系坐标信息数据类型
    // 设置头信息
    ts.header.seq = 100; //序列号
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";
    // 设置子坐标
    ts.child_frame_id = "laser";
    ts.transform.translation.x = 0.2;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.5;
    // 子坐标四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, 0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    // 广播
    broadcaster.sendTransform(ts);
    ros::spin();


    return 0;
}
