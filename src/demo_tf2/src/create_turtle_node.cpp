#include "ros/ros.h"
#include "turtlesim/Spawn.h"

// 创建第二只乌龟
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "create_turtle_node");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    ros::service::waitForService("/spawn");
    turtlesim::Spawn spawn;
    spawn.request.name = "turtle222";
    spawn.request.x = 1.0;
    spawn.request.y = 2.0;
    spawn.request.theta = 3.12;
    bool flag = client.call(spawn);
    if(flag){
        ROS_INFO("turtle create success");
    }
    else{
        ROS_INFO("turtle create failed");
    }
    ros::spin();
    return 0;
}
