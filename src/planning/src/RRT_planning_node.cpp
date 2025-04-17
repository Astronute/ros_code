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
    volatile int width = 0;
    volatile int height = 0;
    volatile int origin_x = 0;
    volatile int origin_y = 0;
    volatile float resolution = 0;
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
    cout << "goal" << init_pos[1] << ',' << init_pos[2] <<endl;
}

/////////////////////////////////////////////// RRT ///////////////////////////////////////////////////
struct Node{
    int x;
    int y;
    Node* parent;
    Node(int _x, int _y):x(_x), y(_y), parent(nullptr){}
};

class RRT{
    public:
    float expand_dis = 3.0;
    float goal_sample_rate = 10.0;
    vector<vector<int>> map;
    int map_w, map_h;
    list<Node*> nodeList;
    list<Node*> path;
    Map m;
    nav_msgs::Path pathmsg;
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber init_pos_sub;
    ros::Subscriber goal_pos_sub;
    ros::Publisher path_pub;
    RRT(){};

    void init(float exp_dis, float rate){
        expand_dis = exp_dis;
        goal_sample_rate = rate;
        map_sub = nh.subscribe("/map", 10, &Map::MapCallback, &m);
        cout <<"get map"<<endl;
        while(!m.height){
            ros::spinOnce();
        }
        map = m.map_data;
        map_w = map[0].size();
        map_h = map.size();
        cout << "map size:" << map_w << "x" << map_h << endl;

        init_pos_sub = nh.subscribe("/initialpose", 10, &InitPoseCallback);
        goal_pos_sub = nh.subscribe("/move_base_simple/goal", 10, &goalPoseCallback);
        path_pub = nh.advertise<nav_msgs::Path>("/path", 10);
    }

    Node* rrtSampleFree(){
        Node* rnd = nullptr;
        float num = rand()%100;
        if(num < goal_sample_rate){
            rnd = new Node(int(goal_pos[1]), int(goal_pos[2]));
        }
        else{
            int rnd_x = (num/100 * (map_w - 1));
            int rnd_y = (num/100 * (map_h - 1));
            rnd = new Node(int(rnd_x), int(rnd_y));
        }
        return rnd;
    }

    Node* getnearestNode(const Node* node){
        Node* node_near = nullptr;
        float min = map_w*map_w + map_h*map_h;
        for(auto n:nodeList){
            float dis = (node->x-n->x)*(node->x-n->x)+(node->y-n->y)*(node->y-n->y);
            if(dis<min){
                node_near = n;
                min = dis;
                //cout << min << " " << dis << endl;
            }
        }
        return node_near;
    }

    Node* createnewNode(Node* near, Node* rnd){
        Node* node_new;
        float dx = rnd->x - near->x;
        float dy = rnd->y - near->y;
        float d = sqrt(dx*dx + dy*dy);
        
        if(d<expand_dis){
            return rnd;
        }
        float exp_x = dx*expand_dis/d + near->x;
        float exp_y = dy*expand_dis/d + near->y;
        node_new = new Node(int(exp_x), int(exp_y));
        return node_new;
    }

    // bresenham
    vector<Node*> connectPoint(Node* near, Node* rnd){
        int x0 = near->x;
        int y0 = near->y;
        int x1 = rnd->x;
        int y1 = rnd->y;
        bool steep = false;
        bool rever = false;
        vector<Node*> res;
        if (abs(x1 - x0) < abs(y1 - y0)) {
            swap(x0, y0);
            swap(x1, y1);
            steep = true;
        }
        if (x0 > x1) {
            swap(x0, x1);
            swap(y0, y1);
            rever = true;
        }
        int dx = x1 - x0;
        int dy = y1 - y0;
        int deltaY = abs(2 * dy);
        int middle = dx;
        int y = y0;
        for (int x = x0; x <= x1; ++x) {
            Node* p;
            if (steep) {
                p = new Node(y, x);
                res.push_back(p);
            }
            else {
                p = new Node(x, y);
                res.push_back(p);
            }
            if (deltaY > middle) {
                middle += abs(2 * dx);
                y += (y1>y0?1:-1);
            }
            deltaY += abs(2 * dy);
        }
        if (rever) {
            reverse(res.begin(), res.end());
        }

        // delete res[0];
        // delete res[res.size()-1];
        res[0] = near;
        res[res.size()-1] = rnd;
        Node* pre = res[0];
        for(int i=1; i<res.size(); ++i){
            res[i]->parent = pre;
            pre = res[i];
            // cout << res[i]->x << " " << res[i]->y << "par:"<<res[i]->parent->x << " " << res[i]->parent->y << endl;
        }
        return res;
    }

    bool isCollision(vector<Node*> path){
        for(auto p:path){
            if(p->x<0||p->x>map.size()-1||
            p->y<0||p->y>map[0].size()-1||
            map[p->y][p->x]==100){
                return true;
            }
        }
        return false;
    }

    float dist_To_goal(Node* node){
        int dx = abs(node->x-goal_pos[1]);
        int dy = abs(node->y-goal_pos[2]);
        float dis = sqrt(dx*dx+dy*dy);
        return dis;
    }

    Node* findPath(Node* startNode, Node* endNode){
        nodeList.push_back(startNode);
        long t=5000;
        bool finish = false;
        while(t--){
            Node* node_rnd = rrtSampleFree();
            // cout << "rnd:" << node_rnd->x << " " << node_rnd->y << endl;
            Node* node_near = getnearestNode(node_rnd);
            // cout << "near:" << node_near->x << " " << node_near->y << endl;
            Node* node_new = createnewNode(node_near, node_rnd);
            if(dist_To_goal(node_new)<expand_dis){
                node_new = new Node(goal_pos[1], goal_pos[2]);
                finish = true;
            }
            
            vector<Node*> line_ = connectPoint(node_near, node_new);
            // draw path
            ros::Time plan_time = ros::Time::now();
            nav_msgs::Path gui_path;
            gui_path.poses.resize(line_.size());
            gui_path.header.frame_id = "map";
            gui_path.header.stamp = plan_time;
        
            // Extract the plan in world co-ordinates, we assume the path is all in the same frame
            for (unsigned int i = 0; i < line_.size(); i++) {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = "map";
                pose.pose.position.x = line_[i]->x*m.resolution;
                pose.pose.position.y = line_[i]->y*m.resolution;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                gui_path.poses[i] = pose;
            }
        
            path_pub.publish(gui_path);
            // for(int i=0; i<line_.size(); ++i){
            //     if(line_[i]!=startNode){
            //         cout << "index:"<< i << " " <<line_[i]->x << " " << line_[i]->y << "par:"<<line_[i]->parent->x << " " << line_[i]->parent->y << endl;
            //     }
            // }
            if(!isCollision(line_)){
                for(auto p:line_){
                    if(p!=node_near){
                        nodeList.push_back(p);
                    }
                }
                cout << "near:" << node_near->x << " " << node_near->y << endl;
                cout << "new:" << node_new->x << " " << node_new->y << endl;
            }
            else{
                cout << "collision" << endl;
                finish = false;
            }

            if(finish){
                cout << "find it" << endl;
                return node_new;
            }
        }
        return nullptr;
    }

    void getPath(){
        path.clear();
        nodeList.clear();
        if(init_pos[0]&&goal_pos[0]){
            init_pos[0] = 0;
            goal_pos[0] = 0;
            init_pos[1] = init_pos[1]/m.resolution;
            init_pos[2] = init_pos[2]/m.resolution;
            goal_pos[1] = goal_pos[1]/m.resolution;
            goal_pos[2] = goal_pos[2]/m.resolution;
            Node* start = new Node((int)init_pos[1], (int)init_pos[2]);
            Node* end = new Node((int)goal_pos[1], (int)goal_pos[2]);
            cout << "start node:" << start->x << " " << start->y << endl;
            Node* res = findPath(start, end);
            while(res){
                path.push_front(res);
                res = res->parent;
            }
            if(nodeList.size()){
                //cout << "size:" << nodeList.size() << endl;
                // for(auto node:nodeList){
                //     path.push_front(node);
                //     cout << "res:" << node->x << " " << node->y << endl;
                // }
                // path.push_front(nodeList.front());
                // path.push_front(nodeList.back());
            }
        }
    }

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        gui_path.header.frame_id = "map";
        gui_path.header.stamp = ros::Time::now();
    
        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }
    
        path_pub.publish(gui_path);
    }
};
//////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "astr_planning_node");
    
    RRT rrt;
    rrt.init(10, 10);
    ros::Rate r(1);
    while(ros::ok()){
        rrt.getPath();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
