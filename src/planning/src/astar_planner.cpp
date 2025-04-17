#include "astar_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mAstar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

namespace mAstar_planner{
    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_WARN("initialize is ok?");
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width*height;
            frame_id_ = costmap_ros->getGlobalFrameID();
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing");
        }
    }

    AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_WARN("is it ok?");
        AstarPlanner::initialize(name, costmap_ros);
    }

    AstarPlanner::AstarPlanner(){}

    // 根据节点index计算节点之间的移动代价
    double AstarPlanner::getMovecost(Node* first, Node* second){
        int dis = abs(static_cast<int>(first->x - second->x)) + abs(static_cast<int>(first->y - second->y));
        if(dis !=1 && dis !=2){
            ROS_ERROR("NODE DISTANCE ERROR");
            return 1;
        }
        if(dis==1){
            return 1;
        }
        else{
            return 1.4;
        }
    }

    float AstarPlanner::G(Node* cur, Node* start){
        return sqrt((cur->x-start->x)*(cur->x-start->x)+(cur->y-start->y)*(cur->y-start->y));
    }

    float AstarPlanner::H(Node* cur, Node* end){
        return (abs(static_cast<int>(cur->x-end->x))+abs(static_cast<int>(cur->y-end->y)));
    }

    Node* AstarPlanner::getLatestnode(list<Node*>& openlist){
        Node * res = openlist.front();
        if(!openlist.empty()){
            for(auto n:openlist){
                if(n->cost<res->cost){
                    res = n;
                }
            }
            return res;
        }
        return nullptr;
    }

    bool AstarPlanner::isCollision(int mx, int my){
        if(mx>costmap_->getSizeInCellsX()||my>costmap_->getSizeInCellsY()){
            return true;
        }
        unsigned char cost = costmap_->getCost(mx, my);
        if(costmap_->getCost(mx, my)>= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            return true;
        }
        return false;
    }

    vector<unsigned int> AstarPlanner::getSurrondnode(const Node* node){
        vector<unsigned int> surrNodes;
        for(int x=node->x-1; x<=node->x+1; ++x){
            for(int y=node->y-1; y<=node->y+1; ++y){
                if(!isCollision(x, y)&&(node->x!=x||node->y!=y)){
                    surrNodes.push_back(costmap_->getIndex(x, y));
                }
            }
        }
        return surrNodes;
    }

    Node* AstarPlanner::isInlist(const list<Node*> l, unsigned int index){
        for(auto p:l){
            if(p->index == index){
                return p;
            }
        }
        return nullptr;
    }

    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan){
        if(!initialized_){
            ROS_WARN("This planner has not been initialized, please call initialize()");
            return false;
        }
        ROS_INFO("Got start node: %.2f, %.2f, goal node: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                goal.pose.position.x, goal.pose.position.y);
        openlist.clear();
        closelist.clear();
        
        Node startNode,goalNode;
        double wx = start.pose.position.x;
        double wy = start.pose.position.y;
        unsigned int mx, my;
        costmap_->worldToMap(wx, wy, startNode.x, startNode.y);
        startNode.index = costmap_->getIndex(startNode.x, startNode.y);
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;
        costmap_->worldToMap(wx, wy, goalNode.x, goalNode.y);
        goalNode.index = costmap_->getIndex(goalNode.x, goalNode.y);
        startNode.g = 0;
        startNode.h = H(&startNode, &goalNode);
        startNode.cost = startNode.g + startNode.h;
        startNode.parent = nullptr;
        openlist.push_back(&startNode);

        int t = 5000;
        plan.clear();
        while(!openlist.empty()&&t--){
            NodePtr curNode = getLatestnode(openlist);
            openlist.remove(curNode);
            closelist.push_back(curNode);
            if(curNode->x==goalNode.x&&curNode->y==goalNode.y){
                NodePtr p = curNode;
                ros::Time plan_time = ros::Time::now();
                while(p){
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = plan_time;
                    pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                    double wx, wy;
                    costmap_->mapToWorld(p->x,p->y, wx, wy);
                    pose.pose.position.x = wx;
                    pose.pose.position.y = wy;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    plan.push_back(pose);
                    p = p->parent;
                }
                ROS_WARN("A* based trajectory planning success!!! size:%d", int(plan.size()));
                reverse(plan.begin(), plan.end());
                publishPlan(plan);
                return true;
            }else{
                NodePtr p = curNode;
                ros::Time plan_time = ros::Time::now();
                while(p){
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = plan_time;
                    pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                    double wx, wy;
                    costmap_->mapToWorld(p->x,p->y, wx, wy);
                    pose.pose.position.x = wx;
                    pose.pose.position.y = wy;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    plan.push_back(pose);
                    p = p->parent;
                }
                ROS_WARN("current size:%d", int(plan.size()));
                reverse(plan.begin(), plan.end());
                publishPlan(plan);
                plan.clear();
            }

            vector<unsigned int> surrondings = getSurrondnode(curNode);
            for(auto index:surrondings){
                if(isInlist(closelist, index)){
                    continue;
                }
                NodePtr p = isInlist(openlist, index);
                if(p){
                    if((getMovecost(curNode, p)+curNode->g)<p->g){
                        p->g = getMovecost(curNode, p)+curNode->g;
                        p->parent = curNode;
                    }
                }
                else{
                    Node* node = new Node;
                    costmap_->indexToCells(index, node->x, node->y);
                    node->index = index;
                    node->parent = curNode;
                    node->g = G(node, &startNode);
                    node->h = H(node, &goalNode);
                    node->cost = node->g+node->h;
                    openlist.push_back(node);
                }
            }
        }
        ROS_WARN("OUT OF TIME");
        return false;
    }

    void AstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();
    
        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }
    
        plan_pub_.publish(gui_path);
    }

}



