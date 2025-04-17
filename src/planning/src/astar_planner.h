#ifndef MY_ASTAR_PLANNER_H_
#define MY_ASTAR_PLANNER_H_

#include <vector>
#include <string>
#include <list>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>


using namespace std;

typedef struct Node{
    unsigned int x, y; // map coordinate
    float cost, g, h; // cost:f=g+h
    unsigned int index;
    Node* parent;
    Node(){};
    Node(int _x, int _y):x(_x), y(_y), cost(FLT_MAX), parent(nullptr){}
}*NodePtr, Node;

namespace mAstar_planner{
    class AstarPlanner:public nav_core::BaseGlobalPlanner{
        public:
        int width;
        int height;
        int map_size;
        string frame_id_;
        bool initialized_;
        list<Node*> openlist;
        list<Node*> closelist;
        AstarPlanner();
        AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        
        double getMovecost(Node* first, Node* second);
        float G(Node* cur, Node* start);
        float H(Node* cur, Node* end);
        Node* getLatestnode(list<Node*>& openlist);
        bool isInBounds(int x, int y);
        bool isCollision(int x, int y);
        vector<unsigned int> getSurrondnode(const Node* node);
        Node* isInlist(const list<Node*> l, unsigned int node);

        private:
        costmap_2d::Costmap2DROS* costmap_ros_;
        double step_size_, min_dist_from_robot_;
        costmap_2d::Costmap2D* costmap_;
        double footprintCost(double x_i, double y_i, double theta_i);
        ros::Publisher plan_pub_;
    };
}

#endif
