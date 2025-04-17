#include <iostream>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include "rs_path.h"
#include <set>
#include <Eigen/Core>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <algorithm>
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace std;
using namespace Eigen;

double wheel_base_ = 0.21; // 轴距
double steering_radian_ = 15 * M_PI / 180; // 最大转向角度
double steering_radian_step_size_ = 15.0 * M_PI / 180.0 / 1.0; // rad
double ANGULAR_RESOLUTION_ = 5.0 * M_PI / 180;
int STATE_GRID_SIZE_PHI_ = 72;
double move_step_size_ = 0.5; // m
double segment_length_ = 1.5; // 扩展距离
double steering_change_penalty_ = 2.0;
double steering_penalty_ = 1.5;
double reversing_penalty_ = 1.5;
double shot_distance_ = 0.5;

class Maap{
public:
    int width = 0;
    int height = 0;
    float origin_x = 0.0;
    float origin_y = 0.0;
    float resolution = 0.0;
    vector<vector<int>> map_data;
    Maap(){}
    
    void MapCallback(const nav_msgs::OccupancyGrid& msg);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
};

typedef struct Node{
    enum DIRECTION {
        FORWARD = 0, BACKWARD = 1, NO = 3
    };
    enum NODE_STATUS {
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
    };
    double g,h,f;
    unsigned int index;
    // 地图坐标系
    unsigned int x,y; 
    int i_yaw; 
    // 世界坐标系
    double w_x, w_y; 
    float yaw;
    DIRECTION direction{};
    NODE_STATUS status{};
    int steering_grade;
    Node* parent;
    Node(){}
    Node(unsigned int _x, unsigned int _y):x(_x), y(_y), f(FLT_MAX), parent(nullptr){}
}Node, *NodePtr;

struct cmp {
	bool operator()(const Node* a, const Node* b) const {
		return a->f < b->f;
	}
};


class hybridAStar{
    public:
    int width;
    int height;
    int map_size;
    NodePtr*** state_node_map_ = nullptr;
    
    string frame_id_;
    bool initialized_;
    set<Node*, cmp> openlist;
    list<Node*> closelist;

    hybridAStar();
    hybridAStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(unsigned width, unsigned int height);

    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
    
    double getMovecost(Node* first, Node* second);

    vector<Node*> Expansion(const Node* cur);

    Node* isInlist(const set<Node*, cmp> l, Node* node);

    float ComputeG(Node* &cur, Node* &neighbor);

    float ComputeH(Node* &cur, Node* &end);

    void DynamicModel(const double &step_size, const double &phi,
        double &x, double &y, double &theta) const;
    
    void SetVehicleShape(double length, double width, double rear_axle_dist);

    bool CheckCollision(const double &x, const double &y, const double &theta);

    bool CheckBoundary(unsigned int x, unsigned int y);

    inline bool LineCheck(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1);

    int Euler2index(double &Euler);

    bool AnalyticExpansions(const Node* &current_node_ptr, const Node* &goal_node_ptr);

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    void Reset();

    private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D* costmap_;
    VectorXd vehicle_shape_;
    std::shared_ptr<RSPath> rs_path_ptr_;
    
    double footprintCost(double x_i, double y_i, double theta_i);
    ros::Publisher plan_pub_;
};

class AStar{
public:
    int width;
    int height;
    int map_size;
    string frame_id_;
    bool initialized_;
    list<Node*> openlist;
    list<Node*> closelist;
    AStar();

    void initMap(const vector<vector<int>>& m);

    vector<Node*> Expansion(const Node* cur);

    Node* isInlist(const list<Node*> l, const Node* node);

    Node* findPath(Node* startNode, Node* endNode);

    float ComputeG(Node* &cur, Node* &neighbor);

    float ComputeH(Node* &cur, Node* &end);

    Node* getLatestnode(list<Node*>& openlist);

    bool CheckCollision(const double &x, const double &y, const double &theta);

    bool CheckBoundary(unsigned int x, unsigned int y);


};
