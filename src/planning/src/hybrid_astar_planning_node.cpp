//#include "../../hybrid_astar_global_planner/src/hybrid_Astar.h"

#include "hybrid_astar_planning_node.h"


float init_pos[3] = {0}; // [0]=1 [1]=x [2]=y
float goal_pos[3] = {0};
geometry_msgs::PoseStamped node_start, node_goal;
Maap map_;
bool map_init = false;
/////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Maap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
{
  if (wx < origin_x || wy < origin_y)
    return false;

  mx = (int)((wx - origin_x) / resolution);
  my = (int)((wy - origin_y) / resolution);

  if (mx < width && my < height)
    return true;

  return false;
}


void Maap::MapCallback(const nav_msgs::OccupancyGrid& msg){
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
    node_start.pose = msg.pose.pose;
    node_start.header = msg.header;

    init_pos[0] = 1;
    init_pos[1] = msg.pose.pose.position.x;
    init_pos[2] = msg.pose.pose.position.y;
    unsigned int mx, my;
    map_.worldToMap(init_pos[1], init_pos[2], mx, my);
    cout << "init" << init_pos[1] << ',' << init_pos[2] <<endl;
    cout << "init" << mx << ',' << my <<endl;
    cout << "collision data:" << map_.map_data[my][mx] << endl;
}

void goalPoseCallback(geometry_msgs::PoseStamped msg){
    node_goal = msg;
    goal_pos[0] = 1;
    goal_pos[1] = msg.pose.position.x;
    goal_pos[2] = msg.pose.position.y;
    cout << "goal" << goal_pos[1] << ',' << goal_pos[2] <<endl;
}

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

////////////////////////////////////////////////////////////////////////////


hybridAStar::hybridAStar(){}

void hybridAStar::initialize(unsigned width, unsigned int height){
    ROS_WARN("initialize is ok?");
    ros::NodeHandle private_nh("hybrid_astar");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    if(!initialized_){
        this->width = width;
        this->height = height;
        this->map_size = width*height;

        //rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_)); // 
        state_node_map_ = new NodePtr**[height];
        for(int i=0; i<height; ++i){
            state_node_map_[i] = new NodePtr*[width];
            for(int j=0; j<width; ++j){
                state_node_map_[i][j] = new NodePtr[STATE_GRID_SIZE_PHI_];
                for(int k=0; k<STATE_GRID_SIZE_PHI_; ++k){
                    state_node_map_[i][j][k] = nullptr;
                }
            }
        }

        SetVehicleShape(0.5, 0.5, 0.25); // 设置车辆形状
        initialized_ = true;
    }
    else{
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
}

float hybridAStar::ComputeH(Node* &cur, Node* &end){
    double h;
    h = abs(int(cur->x - end->x)) + abs(int(cur->y - end->y));
    // double len = rs_path_ptr_->Distance(cur->x, cur->y, cur->yaw, end->x, end->y, end->yaw);
    // h = h<len?len:h;
    return h;
}

float hybridAStar::ComputeG(Node* &cur, Node* &neighbor){
    double g;
    //return sqrt((cur->x-neighbor->x)*(cur->x-neighbor->x)+(cur->y-neighbor->y)*(cur->y-neighbor->y));
    if(neighbor->direction == Node::FORWARD){
        if(neighbor->steering_grade !=cur->steering_grade){
            // 行驶方向改变
            if(neighbor->steering_grade == 0){
                // 损失=扩展距离 * 变方向惩罚 * 转向惩罚
                g = segment_length_ * steering_change_penalty_;
            }
            else{
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        }
        else{
            if(neighbor->steering_grade == 0){
                g = segment_length_;
            }
            else{
                g = segment_length_ * steering_penalty_;
            }
        }
    }
    else{
        if(neighbor->steering_grade !=cur->steering_grade){
            // 行驶方向改变
            if(neighbor->steering_grade == 0){
                // 直行
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            }
            else{
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        }
        else{
            if(neighbor->steering_grade == 0){
                g = segment_length_ * reversing_penalty_;
            }
            else{
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }
    return g;
}

// step_size:车辆移动的距离
// 运动学模型
void hybridAStar::DynamicModel(const double &step_size, const double &phi,
    double &x, double &y, double &theta) const {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}

void hybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist){
    // 初始姿态（0，0，0）位置时轮廓坐标
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);
}

// 轮廓发生碰撞为true
inline bool hybridAStar::LineCheck(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1){
    bool steep = false;
    bool rever = false;
    vector<Node*> res;
    if (abs(int(x1 - x0)) < abs(int(y1 - y0))) {
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
    //auto N = static_cast<unsigned int>(x1 - x0);
    for (double x = x0; x < x1; ++x) {
        unsigned int mx,my;
        if (steep) {
            mx = y;
            my = x;
        }
        else {
            mx = x;
            my = y;
        }
        if(map_.map_data[my][mx]==100||
            map_.map_data[my][mx]==-1||
            mx<0||mx>map_.map_data.size()-1||
            my<0||my>map_.map_data[0].size()-1){
            return true;
        }
        if (deltaY > middle) {
            middle += abs(2 * dx);
            y += (y1 > y0 ? 1.0 : -1.0);
        }
        deltaY += abs(2 * dy);
    }
    return false;
}

bool hybridAStar::CheckBoundary(unsigned int x, unsigned int y){
    if(x>height-1 || x<0 || y>width-1 || y <0){
        return true;
    }
    return false;
}

// 发生碰撞为true
bool hybridAStar::CheckCollision(const double &x, const double &y, const double &theta){
    Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);
    
    // 计算车辆轮廓坐标
    MatrixXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0) = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vector2d(x, y);
    }

    unsigned int x0, y0;
    if(!map_.worldToMap(transformed_vehicle_shape(0, 0), transformed_vehicle_shape(1, 0), x0, y0)){
        return true;
    }
    
    unsigned int x1, y1;
    if(!map_.worldToMap(transformed_vehicle_shape(2, 0), transformed_vehicle_shape(3, 0), x1, y1)){
        return true;
    }

    unsigned int x2, y2;
    if(!map_.worldToMap(transformed_vehicle_shape(4, 0), transformed_vehicle_shape(5, 0), x2, y2)){
        return true;
    }

    unsigned int x3, y3;
    if(!map_.worldToMap(transformed_vehicle_shape(6, 0), transformed_vehicle_shape(7, 0), x3, y3)){
        return true;
    }
    if(CheckBoundary(x0, y0)||CheckBoundary(x1, y1)||CheckBoundary(x2,y2)||CheckBoundary(x3,y3)){
        return true;
    }
    if(LineCheck(x0, y0, x1, y1)||LineCheck(x1, y1, x2, y2)||LineCheck(x2, y3, x3, y3)||LineCheck(x3, y3, x0, y0)){
        return true;
    }
    return false;
}

int hybridAStar::Euler2index(double &Euler){
    int index = std::min(std::max(int((Euler - (-M_PI)) / ANGULAR_RESOLUTION_), 0), 
                STATE_GRID_SIZE_PHI_ - 1);
    return index;
}

Node* hybridAStar::isInlist(const set<Node*, cmp> l, Node* node){
    for(auto p:l){
        if(p->index == node->index && p->i_yaw == node->i_yaw){
            return p;
        }
    }
    return nullptr;
}

vector<Node*> hybridAStar::Expansion(const Node* cur){
    vector<Node*> neighbor_nodes;
    for(int i=-1; i<=1; i++){
        // 搜索偏转角度phi：-15度 0度 15度
        VectorVec3d intermediate_state;
        double x = cur->w_x;
        double y = cur->w_y;
        double theta = cur->yaw;
        const double phi = i * steering_radian_step_size_;
        bool has_obstacle = false;
        
        // forward
        for(int j=0; j<8; j++){
            // 搜索距离：0.2*8米
            DynamicModel(move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));
            if (CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }
        if(!has_obstacle){
            Vec3d node = intermediate_state.back();
            Node* n_exp = new Node();
            map_.worldToMap(node(0,0), node(1,0), n_exp->x, n_exp->y);
            n_exp->w_x = node(0,0);
            n_exp->w_y = node(1,0);
            n_exp->yaw = node(2,0);
            n_exp->i_yaw = Euler2index(node(2,0));
            n_exp->direction = Node::FORWARD;
            n_exp->steering_grade = i;
            neighbor_nodes.push_back(n_exp);
        }

        // backward
        has_obstacle = false;
        intermediate_state.clear();
        x = cur->w_x;
        y = cur->w_y;
        theta = cur->yaw;
        for(int j=0; j<8; j++){
            // 搜索距离：0.2*8米
            DynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));
            if (CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }
        if(!has_obstacle){
            Vec3d node = intermediate_state.back();
            Node* n_exp = new Node();
            map_.worldToMap(node(0,0), node(1,0), n_exp->x, n_exp->y);
            n_exp->w_x = node(0,0);
            n_exp->w_y = node(1,0);
            n_exp->yaw = node(2,0);
            n_exp->i_yaw = Euler2index(node(2,0));
            n_exp->direction = Node::BACKWARD;
            n_exp->steering_grade = i;
            neighbor_nodes.push_back(n_exp);
        }
    }
    return neighbor_nodes;
}

bool hybridAStar::AnalyticExpansions(const Node* &current_node_ptr, const Node* &goal_node_ptr){
    Vec3d cur_node(current_node_ptr->w_x, current_node_ptr->w_y, current_node_ptr->yaw);
    Vec3d goal_node(goal_node_ptr->w_x, goal_node_ptr->w_y, goal_node_ptr->yaw);
    // double length = 0;
    // VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(cur_node, goal_node, move_step_size_, length);
    // // mx>costmap_->getSizeInCellsX()||my>costmap_->getSizeInCellsY()
    // for (const auto &pose: rs_path_poses)
    //     if (CheckCollision(pose.x(), pose.y(), pose.z())) {
    //         return false;
    // };

    // goal_node_ptr->intermediate_states_ = rs_path_poses;
    // goal_node_ptr->parent_node_ = current_node_ptr;

    // auto begin = goal_node_ptr->intermediate_states_.begin();
    // goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}

bool hybridAStar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan){
    if(!initialized_){
        ROS_WARN("This planner has not been initialized, please call initialize()");
        return false;
    }
    ROS_INFO("Got start node: %.2f, %.2f, goal node: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);
    openlist.clear();
    closelist.clear();
    Reset();
    Node* startNode = new Node();
    Node* goalNode = new Node();

    tf::Quaternion quat;
    double roll, pitch, yaw;//定义存储r\p\y的容器
    startNode->w_x = start.pose.position.x;
    startNode->w_y = start.pose.position.y;
    map_.worldToMap(startNode->w_x, startNode->w_y, startNode->x, startNode->y);
    tf::quaternionMsgToTF(start.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    startNode->yaw = yaw;
    startNode->i_yaw = Euler2index(yaw);


    goalNode->w_x = goal.pose.position.x;
    goalNode->w_y = goal.pose.position.y;
    map_.worldToMap(goalNode->w_x, goalNode->w_y, goalNode->x, goalNode->y);
    tf::quaternionMsgToTF(goal.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    goalNode->yaw = yaw;
    goalNode->i_yaw = Euler2index(yaw);

    startNode->g = 0;
    startNode->h = ComputeH(startNode, goalNode);
    startNode->f = startNode->g + startNode->h;
    startNode->direction = Node::NO;
    startNode->steering_grade = 0;
    startNode->parent = nullptr;
    state_node_map_[startNode->x][startNode->y][startNode->i_yaw] = startNode;
    state_node_map_[goalNode->x][goalNode->y][goalNode->i_yaw] = goalNode;
    openlist.insert(startNode);
    plan.clear();
    int STEP = 50000;
    while(!openlist.empty()&&STEP--){
        NodePtr curNode = *openlist.begin();
        curNode->status = Node::IN_CLOSESET;
        openlist.erase(openlist.begin());
        
        // 接近终点
        double dist2goal = sqrt((curNode->x-goalNode->x)*(curNode->x-goalNode->x)+(curNode->y-goalNode->y)*(curNode->y-goalNode->y));
        if(dist2goal < shot_distance_){
            
            NodePtr p = curNode;
            ros::Time plan_time = ros::Time::now();
            while(p){
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = "map";
                pose.pose.position.x = p->w_x;
                pose.pose.position.y = p->w_y;
                pose.pose.position.z = 0.0;
                geometry_msgs::Quaternion q;
                q=tf::createQuaternionMsgFromRollPitchYaw(0,0,p->yaw);
                pose.pose.orientation.x = q.x;
                pose.pose.orientation.y = q.y;
                pose.pose.orientation.z = q.z;
                pose.pose.orientation.w = q.w;

                plan.push_back(pose);
                p = p->parent;
            }
            ROS_WARN("Hybrid A* based trajectory planning success!!! size:%d", int(plan.size()));
            reverse(plan.begin(), plan.end());
            publishPlan(plan);
            return true;
        }else{
            NodePtr p = curNode;
            ros::Time plan_time = ros::Time::now();
            while(p){
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = "map";
                pose.pose.position.x = p->w_x;
                pose.pose.position.y = p->w_y;
                pose.pose.position.z = 0.0;
                geometry_msgs::Quaternion q;
                q=tf::createQuaternionMsgFromRollPitchYaw(0,0,p->yaw);
                pose.pose.orientation.x = q.x;
                pose.pose.orientation.y = q.y;
                pose.pose.orientation.z = q.z;
                pose.pose.orientation.w = q.w;
                plan.push_back(pose);
                p = p->parent;
            }
            reverse(plan.begin(), plan.end());
            publishPlan(plan);
            plan.clear();
        }

        // 扩展节点，加入openlist
        vector<Node*> surrondings = Expansion(curNode);
        for(auto neighbor: surrondings){
            const double edge_g = ComputeG(curNode, neighbor);
            const double neighbor_h = ComputeH(curNode, goalNode);
            NodePtr nodeInstatemap = state_node_map_[neighbor->x][neighbor->y][neighbor->i_yaw];
            if(nodeInstatemap==nullptr){
                neighbor->status = Node::IN_OPENSET;
                neighbor->parent =curNode;
                neighbor->g = curNode->g + edge_g;
                neighbor->h = neighbor_h;
                neighbor->f = neighbor->g + neighbor->h;
                state_node_map_[neighbor->x][neighbor->y][neighbor->i_yaw] = neighbor;
                openlist.insert(neighbor);
            }
            else if(nodeInstatemap->status==Node::IN_CLOSESET){
                delete neighbor;
                continue;
            }
            else{
                if(curNode->g + edge_g < nodeInstatemap->g){
                    neighbor->status = Node::IN_OPENSET;
                    nodeInstatemap->g = curNode->g + edge_g;
                    nodeInstatemap->f = nodeInstatemap->g + nodeInstatemap->h;
                    nodeInstatemap->parent = curNode;
                }
                else{
                    delete neighbor;
                }
            }
        }
    }
    cout << "planning failled" << endl;
    return false;
}

void hybridAStar::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

void hybridAStar::Reset(){
    if(state_node_map_){
        for(int i=0; i<height; ++i){
            if(state_node_map_[i]==nullptr){
                continue;
            }
            for(int j=0; j<width; ++j){
                if(state_node_map_[i][j]==nullptr){
                    continue;
                }
                for(int k=0; k<STATE_GRID_SIZE_PHI_; ++k){
                    if(state_node_map_[i][j][k] != nullptr){
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }
}




int main(int argc, char *argv[]){
    ros::init(argc, argv, "astr_planning_node");
    
    hybridAStar hastar = hybridAStar();

    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 10, &Maap::MapCallback, &map_);
    ros::Subscriber init_pos_sub = nh.subscribe("/initialpose", 10, &InitPoseCallback);
    ros::Subscriber goal_pos_sub = nh.subscribe("/move_base_simple/goal", 10, &goalPoseCallback);
    nh.getParam("/steering_change_penalty", steering_change_penalty_);
    nh.getParam("/steering_penalty", steering_penalty_);
    nh.getParam("/reversing_penalty", reversing_penalty_);
    nh.getParam("shot_distance", shot_distance_);
    bool fonce=false;
    ros::Rate r(1);
    while(ros::ok()){
        if(map_.height>0&&map_.width>0&&map_init==false){
            hastar.initialize(map_.width, map_.height);
            map_init = true;
        }
        if(init_pos[0]&&goal_pos[0]&&fonce==false){
            init_pos[0] = 0;
            goal_pos[0] = 0;
            vector<geometry_msgs::PoseStamped> path;
            hastar.makePlan(node_start, node_goal, path);
            //path_pub.publish(pathmsg);
            //fonce=true;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}