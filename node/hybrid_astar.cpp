#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>

// --- 常量定义 ---
const double XY_RESOLUTION = 0.5;
const double THETA_RESOLUTION = 0.26;
const int STEERING_INPUTS = 5;
const double MAX_STEER = 0.4;
const double VEHICLE_L = 0.33;
const double MOVEMENT_STEP = 0.5;

// 【新增】倒车相关的惩罚代价
const double REVERSE_COST = 3.0;      // 倒车比直行贵3倍
const double GEAR_SWITCH_COST = 5.0;  // 换挡一次代价很高，防止频繁震荡
const double STEER_COST = 0.5;        // 打方向盘的代价
const double STEER_CHANGE_COST = 0.5; // 急打方向盘的代价

struct Node3D {
    double x, y, theta;
    int x_idx, y_idx, theta_idx;
    double g, h, f;
    Node3D* parent;
    double steer;
    int direction; // 1 = 前进, -1 = 后退

    void compute_index() {
        x_idx = std::round(x / XY_RESOLUTION);
        y_idx = std::round(y / XY_RESOLUTION);
        double t = theta;
        while (t < 0) t += 2 * M_PI;
        while (t >= 2 * M_PI) t -= 2 * M_PI;
        theta_idx = std::round(t / THETA_RESOLUTION);
    }
    
    long long get_key() const {
        return ((long long)x_idx << 32) | ((long long)y_idx << 16) | theta_idx;
    }
};

struct CompareNode {
    bool operator()(const Node3D* a, const Node3D* b) {
        return a->f > b->f;
    }
};

class HybridAStar {
private:
    ros::NodeHandle n;
    ros::Subscriber map_sub, goal_sub, odom_sub;
    ros::Publisher path_pub;
    
    nav_msgs::OccupancyGrid map_data;
    bool map_received = false;
    double car_x, car_y, car_yaw;
    
    std::vector<Node3D*> all_nodes;

public:
    HybridAStar() {
        map_sub = n.subscribe("/map", 1, &HybridAStar::map_cb, this);
        goal_sub = n.subscribe("/move_base_simple/goal", 1, &HybridAStar::goal_cb, this);
        odom_sub = n.subscribe("/odom", 1, &HybridAStar::odom_cb, this);
        path_pub = n.advertise<nav_msgs::Path>("/planned_path", 1);
        
        ROS_INFO("Hybrid A* (With Reverse) Initialized.");
    }

    ~HybridAStar() {
        for (auto n : all_nodes) delete n;
    }

    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_data = *msg;
        map_received = true;
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        car_x = msg->pose.pose.position.x;
        car_y = msg->pose.pose.position.y;
        car_yaw = tf::getYaw(msg->pose.pose.orientation);
    }

    void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!map_received) {
            ROS_WARN("No map received yet!");
            return;
        }
        
        for (auto n : all_nodes) delete n;
        all_nodes.clear();

        double goal_x = msg->pose.position.x;
        double goal_y = msg->pose.position.y;

        ROS_INFO("Planning start... (Reverse Allowed)");
        
        std::priority_queue<Node3D*, std::vector<Node3D*>, CompareNode> open_list;
        std::unordered_map<long long, Node3D*> closed_list;

        // 起点 (direction 初始设为 1，即假设当前是挂D档的)
        Node3D* start_node = new Node3D{car_x, car_y, car_yaw, 0, 0, 0, 0, 0, 0, nullptr, 0.0, 1};
        start_node->compute_index();
        start_node->g = 0.0;
        start_node->h = hypot(car_x - goal_x, car_y - goal_y);
        start_node->f = start_node->g + start_node->h;
        
        open_list.push(start_node);
        all_nodes.push_back(start_node);
        closed_list[start_node->get_key()] = start_node;

        Node3D* final_node = nullptr;
        int iter = 0;
        int max_iter = 25000; // 因为搜索空间变大了(x2)，增加迭代次数上限

        while (!open_list.empty() && iter < max_iter) {
            iter++;
            Node3D* current = open_list.top();
            open_list.pop();

            if (hypot(current->x - goal_x, current->y - goal_y) < 1.0) {
                ROS_INFO("Path Found! Iterations: %d", iter);
                final_node = current;
                break;
            }

            // 【核心修改】两层循环：先遍历方向(前/后)，再遍历转向角
            // dir = 1 (前进), dir = -1 (后退)
            for (int dir = -1; dir <= 1; dir += 2) { 
                
                for (int i = 0; i < STEERING_INPUTS; i++) {
                    double delta = -MAX_STEER + i * (2.0 * MAX_STEER / (STEERING_INPUTS - 1));
                    
                    // 运动学方程：根据方向决定移动距离的正负
                    double distance = dir * MOVEMENT_STEP; 
                    
                    double beta = atan(0.5 * tan(delta));
                    double next_x = current->x + distance * cos(current->theta + beta);
                    double next_y = current->y + distance * sin(current->theta + beta);
                    // 注意：倒车时角度变化也遵循同样的几何关系，distance为负即可
                    double next_theta = current->theta + (distance * tan(delta) / VEHICLE_L);
                    
                    while (next_theta < 0) next_theta += 2 * M_PI;
                    while (next_theta >= 2 * M_PI) next_theta -= 2 * M_PI;

                    if (is_collision(next_x, next_y)) continue;

                    Node3D* next_node = new Node3D{next_x, next_y, next_theta, 0, 0, 0, 0, 0, 0, current, delta, dir};
                    next_node->compute_index();

                    long long key = next_node->get_key();
                    if (closed_list.find(key) != closed_list.end()) {
                        delete next_node; 
                        continue;
                    }

                    // 【核心修改】代价计算 g(n)
                    double cost_g = current->g;
                    cost_g += MOVEMENT_STEP; // 基础移动代价
                    
                    // 1. 倒车惩罚
                    if (dir == -1) cost_g += REVERSE_COST;
                    
                    // 2. 换挡惩罚 (如果这次方向跟上次不一样)
                    if (dir != current->direction) cost_g += GEAR_SWITCH_COST;
                    
                    // 3. 转向惩罚
                    cost_g += std::abs(delta) * STEER_COST;
                    
                    // 4. 打舵变化惩罚
                    cost_g += std::abs(delta - current->steer) * STEER_CHANGE_COST;

                    next_node->g = cost_g;
                    next_node->h = hypot(next_x - goal_x, next_y - goal_y);
                    next_node->f = next_node->g + next_node->h;

                    open_list.push(next_node);
                    all_nodes.push_back(next_node);
                    closed_list[key] = next_node;
                }
            }
        }

        if (final_node) publish_path(final_node);
        else ROS_WARN("Hybrid A* Failed!");
    }

    bool is_collision(double x, double y) {
        if (!map_received) return true;
        int mx = (int)((x - map_data.info.origin.position.x) / map_data.info.resolution);
        int my = (int)((y - map_data.info.origin.position.y) / map_data.info.resolution);
        if (mx < 0 || my < 0 || mx >= map_data.info.width || my >= map_data.info.height) return true;
        
        int inflation = 0.5 / map_data.info.resolution; 
        for (int dx = -inflation; dx <= inflation; dx++) {
            for (int dy = -inflation; dy <= inflation; dy++) {
                int index = (my + dy) * map_data.info.width + (mx + dx);
                if (index >= 0 && index < map_data.data.size()) {
                    if (map_data.data[index] > 50) return true; 
                }
            }
        }
        return false;
    }

    void publish_path(Node3D* node) {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        while (node != nullptr) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = node->x;
            pose.pose.position.y = node->y;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(node->theta);
            path.poses.push_back(pose);
            node = node->parent;
        }
        std::reverse(path.poses.begin(), path.poses.end());
        path_pub.publish(path);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "hybrid_astar_node");
    HybridAStar planner;
    ros::spin();
    return 0;
}