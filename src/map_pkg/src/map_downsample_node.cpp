#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
float init_pos[3] = {0}; // [0]=1 [1]=x [2]=y
float goal_pos[3] = {0};
geometry_msgs::PoseStamped node_start, node_goal;
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class Map_{
public:
    int width = 0;
    int height = 0;
    float origin_x = 0.0;
    float origin_y = 0.0;
    float resolution = 0.0;
    vector<vector<int>> map_data;
    vector<vector<int>> map_down2;
    Map_(){}
    
    void MapCallback(const nav_msgs::OccupancyGrid& msg);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    void InitPoseCallback(geometry_msgs::PoseWithCovarianceStamped msg);
    void goalPoseCallback(geometry_msgs::PoseStamped msg);
    bool mapDownSample(vector<vector<int>> &map);
    void pubDwonSample();

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map_d2", 10);
    bool origin_map_init_ = false;
    bool d2_map_init = false;
};

bool Map_::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my){
    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (int)((wx - origin_x) / resolution);
    my = (int)((wy - origin_y) / resolution);

    if (mx < width && my < height)
        return true;

    return false;
}

void Map_::MapCallback(const nav_msgs::OccupancyGrid& msg){
    if(!origin_map_init_){
        width = msg.info.width;
        height = msg.info.height;
        origin_x = msg.info.origin.position.x;
        origin_y = msg.info.origin.position.y;
        resolution = msg.info.resolution;
        map_data = vector<vector<int>>(height, vector<int>(width, 0));
        for(int i=0; i<height; ++i){
            for(int j=0; j<width; ++j){
                map_data[i][j] = msg.data[i*width+j];
            }
        }
        origin_map_init_ = true;
    }
}

void Map_::InitPoseCallback(geometry_msgs::PoseWithCovarianceStamped msg){
    node_start.pose = msg.pose.pose;
    node_start.header = msg.header;

    init_pos[0] = 1;
    init_pos[1] = msg.pose.pose.position.x;
    init_pos[2] = msg.pose.pose.position.y;
    unsigned int mx, my;
    worldToMap(init_pos[1], init_pos[2], mx, my);
    cout << "init" << init_pos[1] << ',' << init_pos[2] <<endl;
    cout << "init" << mx << ',' << my <<endl;
    cout << "collision data:" << map_data[my][mx] << endl;
}

void Map_::goalPoseCallback(geometry_msgs::PoseStamped msg){
    node_goal = msg;
    goal_pos[0] = 1;
    goal_pos[1] = msg.pose.position.x;
    goal_pos[2] = msg.pose.position.y;
    cout << "goal" << goal_pos[1] << ',' << goal_pos[2] <<endl;
}

bool Map_::mapDownSample(vector<vector<int>>& map_data_){
    if(origin_map_init_){
        map_down2 = vector<vector<int>>(height/2+height%2, vector<int>(width/2+width%2, 0));
        for(int x=0; x<height; ++x){
            if(x%2==0){
                for(int y=0; y<width; ++y){
                    if(y%2==0){
                        map_down2[y/2][x/2] = map_data_[y][x];
                    }
                }
            }
        }
        d2_map_init = true;
        return true;
    }
    return false;
}

void Map_::pubDwonSample(){
// publish
    if(map_down2.size()){
        nav_msgs::OccupancyGrid msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.info.origin.position.x = origin_x;
        msg.info.origin.position.y = origin_y;
        msg.info.resolution = resolution * 2;
        msg.info.width = map_down2[0].size();
        msg.info.height = map_down2.size();
        msg.data.resize(msg.info.width * msg.info.height);
        for(int x=0; x<msg.info.width; ++x){
            for(int y=0; y<msg.info.height; ++y){
                msg.data[x * msg.info.width + y] = map_down2[x][y];
            }
        }
        pub.publish(msg);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "astr_planning_node");
    Map_ map_;
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 10, &Map_::MapCallback, &map_);
    ros::Subscriber init_pos_sub = nh.subscribe("/initialpose", 10, &Map_::InitPoseCallback, &map_);
    ros::Subscriber goal_pos_sub = nh.subscribe("/move_base_simple/goal", 10, &Map_::goalPoseCallback, &map_);

    ros::Rate r(1);
    while(ros::ok()){
        if(!map_.d2_map_init){
            map_.mapDownSample(map_.map_data);
        }
        else{
            map_.pubDwonSample();
        }
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
