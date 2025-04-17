#include "ros/ros.h"
#include "iostream"

using namespace std;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "demo_param_node");
    ros::NodeHandle nh;
    int param_int;
    float param_float;
    string param_str;
    int param_l0;

    nh.getParam("/param_int", param_int);
    nh.getParam("/param_float", param_float);
    nh.getParam("/param_str", param_str);
    nh.getParam("param_l0", param_l0);

    cout << param_l0 << " " << endl;
    // cout << param_str << endl;
    while(ros::ok()){
        
    }
    return 0;
}
