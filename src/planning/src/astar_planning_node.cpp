#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <list>
#include <iostream>

using namespace std;
float init_pos[3] = {0}; // [0]=1 [1]=x [2]=y
float goal_pos[3] = {0};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
class Map{
    public:
    int width = 0;
    int height = 0;
    int origin_x = 0;
    int origin_y = 0;
    float resolution = 0;
    vector<vector<int>> map_data;
    Map(){}
    
    void MapCallback(const nav_msgs::OccupancyGrid& msg);
};


void Map::MapCallback(const nav_msgs::OccupancyGrid& msg){
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
}

void InitPoseCallback(geometry_msgs::PoseWithCovarianceStamped msg){
    init_pos[0] = 1;
    init_pos[1] = msg.pose.pose.position.x;
    init_pos[2] = msg.pose.pose.position.y;
    cout << "init" << init_pos[1] << ',' << init_pos[2] <<endl;
}

void goalPoseCallback(geometry_msgs::PoseStamped msg){
    goal_pos[0] = 1;
    goal_pos[1] = msg.pose.position.x;
    goal_pos[2] = msg.pose.position.y;
    cout << "goal" << goal_pos[1] << ',' << goal_pos[2] <<endl;
}

/////////////////////////////////////////////// A* ///////////////////////////////////////////////////
struct Node{
    int x;
    int y;
    float f, g, h;
    Node* parent;
    Node(int _x, int _y):x(_x), y(_y), f(0), g(0), h(0), parent(nullptr){}
};

class A_star{
    public:
    int start_x;
    int start_y;
    int target_x;
    int target_y;
    //Node* startNode, endNode;
    list<Node*> openlist;
    list<Node*> closelist;
    vector<vector<int>> map;
    bool flag_map = false;

    A_star(){};

    void initMap(const vector<vector<int>>& m){
        map = m;
    }
    // F=G+H
    float G(Node* cur, Node* start){
        return sqrt((cur->x-start->x)*(cur->x-start->x)+(cur->y-start->y)*(cur->y-start->y));
    }

    float H(Node* cur, Node* end){
        return (abs(cur->x-end->x)+abs(cur->y-end->y));
    }

    Node* getLatestnode(list<Node*>& openlist){
        Node * res = openlist.front();
        if(!openlist.empty()){
            for(auto n:openlist){
                if(n->f<res->f){
                    res = n;
                }
            }
            return res;
        }
        return nullptr;
    }

    bool isCanreach(const Node* target, const Node* node){
        if(target->x<0||target_x>map.size()-1||
        target->y<0||target_y>map[0].size()-1||
        map[target->y][target->x]==100||
        (target->x==node->x&&target->y==node->y)||
        isInlist(closelist, target)){
            return false;
        }
        else{
            return true;
        }
    }

    vector<Node*> getSurrondnode(const Node* node){
        vector<Node*> sNodes;
        for(int x=node->x-1; x<=node->x+1; ++x){
            for(int y=node->y-1; y<=node->y+1; ++y){
                if(isCanreach(new Node(x,y), node)){
                    sNodes.push_back(new Node(x,y));
                }
            }
        }
        return sNodes;
    }

    Node* isInlist(const list<Node*> l, const Node* node){
        for(auto p:l){
            if(p->x==node->x&&p->y==node->y){
                return p;
            }
        }
        return nullptr;
    }

    Node* findPath(Node* startNode, Node* endNode){
        startNode->g = 0;
        startNode->h = H(startNode, endNode);
        startNode->f = startNode->g + startNode->h;
        startNode->parent = nullptr;
        openlist.push_back(startNode);
        long t=10000;
        while(!openlist.empty()&&t--){
            Node* curNode = getLatestnode(openlist);
            openlist.remove(curNode);
            closelist.push_back(curNode);
            if(curNode->x==endNode->x&&curNode->y==endNode->y){
                cout << "find it" << curNode->x << " " << curNode->y << endl;
                return curNode;
            }
            // 
            vector<Node*> surrondingNodes =  getSurrondnode(curNode);
            for(Node* node:surrondingNodes){
                Node* p = isInlist(openlist, node); 
                if(p){
                    node->g = G(startNode, node);
                    if(node->g<p->g){
                        p->g = node->g;
                        p->parent = curNode;
                    }
                }
                else{
                    node->parent = curNode;
                    node->g = G(node, startNode);
                    node->h = H(node, endNode);
                    node->f = node->g+node->h;
                    openlist.push_back(node);
                }
            }
        }
        cout << "filed" << endl;
        return nullptr;
    }

    list<Node*> getPath(Node* startNode, Node* endNode){
        Node* res = findPath(startNode, endNode);
        list<Node*> path;
        while(res){
            path.push_front(res);
            res = res->parent;
        }
        openlist.clear();
        closelist.clear();
        return path;
    }

};
//////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "astr_planning_node");
    cout << "astar planner start" << endl;
    Map map;
    A_star astar;
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 10, &Map::MapCallback, &map);
    ros::Subscriber init_pos_sub = nh.subscribe("/initialpose", 10, &InitPoseCallback);
    ros::Subscriber goal_pos_sub = nh.subscribe("/move_base_simple/goal", 10, &goalPoseCallback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 10);

    nav_msgs::Path pathmsg;
    pathmsg.header.frame_id = "map";
    pathmsg.header.stamp = ros::Time::now();
    bool fonce=false;
    ros::Rate r(1);
    while(ros::ok()){
        if(map.height>0&&map.width>0&&astar.flag_map==false){
            astar.initMap(map.map_data);
            astar.flag_map = true;
        }
        if(init_pos[0]&&goal_pos[0]&&fonce==false){
            init_pos[0] = 0;
            goal_pos[0] = 0;
            init_pos[1] = init_pos[1]/map.resolution;
            init_pos[2] = init_pos[2]/map.resolution;
            goal_pos[1] = goal_pos[1]/map.resolution;
            goal_pos[2] = goal_pos[2]/map.resolution;
            Node* start = new Node((int)init_pos[1], (int)init_pos[2]);
            Node* end = new Node((int)goal_pos[1], (int)goal_pos[2]);
            list<Node*> path = astar.getPath(start, end);
            pathmsg.poses.clear();
            for(auto p: path){
                geometry_msgs::PoseStamped ps;
                ps.header.frame_id = "map";
                ps.header.stamp = ros::Time::now();
                ps.pose.position.x = p->x*map.resolution;
                ps.pose.position.y = p->y*map.resolution;
                ps.pose.orientation.x = 0;
                ps.pose.orientation.y = 0;
                ps.pose.orientation.z = 0;
                ps.pose.orientation.w = 0;
                pathmsg.poses.push_back(ps);
                path_pub.publish(pathmsg);
            }
            //path_pub.publish(pathmsg);
            //fonce=true;
        }
        path_pub.publish(pathmsg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
