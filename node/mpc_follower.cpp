#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h> // 新增：需要读地图
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

// --- MPC 参数配置 (调参区) ---
const int N_SAMPLES = 150;      // 增加采样数，让它多试几种可能
const int PREDICT_STEPS = 12;   // 步长不用太长，1.2秒足够
const double DT = 0.1;          
const double MAX_STEER = 0.41;  
const double WHEELBASE = 0.33;  

// 权重参数
const double WEIGHT_TRACKING = 1.0;  
const double WEIGHT_HEADING = 2.0;   
const double WEIGHT_STEER = 1.5;     // 加大转向惩罚，让它别乱扭
const double WEIGHT_COLLISION = 10000.0; // 【新增】撞墙代价，一票否决

struct State {
    double x;
    double y;
    double yaw;
    double v;
};

struct Point {
    double x, y;
};

class MPCFollower {
private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Publisher traj_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber path_sub;
    ros::Subscriber map_sub; // 【新增】
    
    std::vector<Point> path_points;
    nav_msgs::OccupancyGrid map_data; // 【新增】
    bool map_received = false;
    int last_closest_index = 0;
    
    std::default_random_engine generator;
    std::normal_distribution<double> distribution{0.0, 0.25}; // 减小一点噪声

public:
    MPCFollower() {
        path_sub = n.subscribe("/planned_path", 1, &MPCFollower::path_cb, this);
        odom_sub = n.subscribe("/odom", 1, &MPCFollower::odom_cb, this);
        map_sub = n.subscribe("/map", 1, &MPCFollower::map_cb, this); // 【新增】
        
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
        traj_pub = n.advertise<visualization_msgs::MarkerArray>("/mpc_trajectories", 1);
        
        ROS_INFO("Safety-Aware MPC Follower Initialized!");
    }

    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_data = *msg;
        map_received = true;
    }

    void path_cb(const nav_msgs::Path::ConstPtr& msg) {
        path_points.clear();
        for (const auto& pose : msg->poses) {
            path_points.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        last_closest_index = 0;
    }

// 【修改后】带膨胀半径的碰撞检测
    bool is_collision(double x, double y) {
        if (!map_received) return false;
        
        // 1. 计算中心点的栅格坐标
        int mx = (int)((x - map_data.info.origin.position.x) / map_data.info.resolution);
        int my = (int)((y - map_data.info.origin.position.y) / map_data.info.resolution);
        
        // 越界检查
        if (mx < 0 || my < 0 || mx >= map_data.info.width || my >= map_data.info.height) return true;

        // 2. 【核心修改】膨胀检测循环
        // 设定膨胀半径：0.3米 (根据F1TENTH车宽调整，稍微大一点更安全)
        // 意思是：车子不仅中心不能撞，周围 0.3 米内也不能有墙
        int inflation_radius_cells = 0.50 / map_data.info.resolution; 

        for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; dx++) {
            for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; dy++) {
                int check_x = mx + dx;
                int check_y = my + dy;
                
                // 再次检查边界
                if (check_x >= 0 && check_x < map_data.info.width && 
                    check_y >= 0 && check_y < map_data.info.height) {
                    
                    int index = check_y * map_data.info.width + check_x;
                    
                    // 如果发现障碍物 (值 > 50)，直接返回 true (撞了)
                    if (map_data.data[index] > 50) return true; 
                }
            }
        }

        return false; // 检查了一圈都没撞，才是真的安全
    }

    State update_state(State s, double steer, double v, double dt) {
        State next_s;
        next_s.x = s.x + v * cos(s.yaw) * dt;
        next_s.y = s.y + v * sin(s.yaw) * dt;
        next_s.yaw = s.yaw + (v * tan(steer) / WHEELBASE) * dt;
        next_s.v = v;
        return next_s;
    }

    int get_closest_index(State vehicle_state) {
        double min_dist = 1e9;
        int idx = last_closest_index;
        for (int i = 0; i < 50; i++) { 
            int curr = (last_closest_index + i);
            if (curr >= path_points.size()) break;
            double d = hypot(path_points[curr].x - vehicle_state.x, 
                             path_points[curr].y - vehicle_state.y);
            if (d < min_dist) {
                min_dist = d;
                idx = curr;
            }
        }
        return idx;
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        if (path_points.empty()) return;

        State current_state;
        current_state.x = msg->pose.pose.position.x;
        current_state.y = msg->pose.pose.position.y;
        current_state.yaw = tf::getYaw(msg->pose.pose.orientation);
        
        last_closest_index = get_closest_index(current_state);
        
        int target_idx = last_closest_index + 8; // 稍微看远一点
        if (target_idx >= path_points.size()) target_idx = path_points.size() - 1;
        
        double dx = path_points[target_idx].x - current_state.x;
        double dy = path_points[target_idx].y - current_state.y;
        double local_x = cos(-current_state.yaw) * dx - sin(-current_state.yaw) * dy;
        double target_v = (local_x >= 0) ? 2.5 : -1.0; 

        std::vector<std::vector<State>> trajectories;
        std::vector<double> costs;
        std::vector<double> first_steers;

        for (int i = 0; i < N_SAMPLES; i++) {
            std::vector<State> traj;
            State temp_state = current_state;
            double cost = 0.0;
            bool collision_flag = false;
            
            double sample_steer = (i == 0) ? 0.0 : distribution(generator); 
            if (sample_steer > MAX_STEER) sample_steer = MAX_STEER;
            if (sample_steer < -MAX_STEER) sample_steer = -MAX_STEER;
            first_steers.push_back(sample_steer);

            for (int t = 0; t < PREDICT_STEPS; t++) {
                temp_state = update_state(temp_state, sample_steer, target_v, DT);
                traj.push_back(temp_state);
                
                // 【核心修改】如果在预测的未来撞墙了，这条轨迹直接废掉
                if (is_collision(temp_state.x, temp_state.y)) {
                    collision_flag = true;
                    break; // 撞了就不用往后算了
                }
            }
            
            if (collision_flag) {
                cost += WEIGHT_COLLISION; // 加上巨额惩罚
            } else {
                // 只有没撞墙的轨迹，才计算循迹误差
                int final_closest = get_closest_index(temp_state);
                double dist_error = hypot(temp_state.x - path_points[final_closest].x,
                                          temp_state.y - path_points[final_closest].y);
                cost += dist_error * WEIGHT_TRACKING;
                cost += std::abs(sample_steer) * WEIGHT_STEER;
            }
            
            trajectories.push_back(traj);
            costs.push_back(cost);
        }

        int best_idx = 0;
        double min_cost = 1e9;
        for (int i = 0; i < N_SAMPLES; i++) {
            if (costs[i] < min_cost) {
                min_cost = costs[i];
                best_idx = i;
            }
        }

        //发布控制 (速度策略大升级) ---
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        
        // 获取 MPC 算出来的最优转向角
        double best_steer = first_steers[best_idx];
        drive_msg.drive.steering_angle = best_steer;

        // --- A. 动态过弯逻辑 (解决拐弯太快) ---
        // 之前的参数太温和了，我们改激进一点
        double max_speed = 4.0;    // 直道极限速度
        double min_speed = 0.6;    // 过弯最低速度 (原来是1.0，改小一点让它能慢下来)
        double speed_gain = 15.0;   // 减速敏感度 (原来是2.0，改大！一打方向就重刹)

        double final_speed = 0.0;

        if (target_v > 0) { // 如果是前进模式
            // 速度 = 极速 - (转向角 * 敏感度)
            // 例如：转向角 0.4(满舵) -> 4.0 - 0.4*6.0 = 1.6 m/s
            final_speed = max_speed - speed_gain * std::abs(best_steer);
            
            // 兜底：无论怎么减速，不能低于 min_speed (防止卡死)
            if (final_speed < min_speed) final_speed = min_speed;
        } else {
            // 倒车模式：保持恒定慢速
            final_speed = -1.0;
        }

        // --- B. 终点停车逻辑 (解决冲过头) ---
        // 计算离路径终点的直线距离
        if (!path_points.empty()) {
            double dist_to_goal = std::hypot(path_points.back().x - current_state.x, 
                                             path_points.back().y - current_state.y);
            
            double stop_dist = 0.2; // 刹停距离
            double slow_dist = 1.5; // 减速开始距离

            if (dist_to_goal < stop_dist) {
                final_speed = 0.0; // 彻底刹停
                ROS_INFO_THROTTLE(1.0, "Goal Reached! Stopping.");
            } 
            else if (dist_to_goal < slow_dist) {
                // 线性减速公式：距离越近，速度越慢
                // 比如距离 0.75m 时，速度限制为原来的 50%
                double ramp_speed = (dist_to_goal / slow_dist) * 2.0; 
                
                // 保证有个“蠕行速度”慢慢挪过去，别直接停了
                if (ramp_speed < 0.3) ramp_speed = 0.3;

                // 取最小值：如果动态速度比减速要求快，就服从减速要求
                if (target_v > 0) {
                     if (final_speed > ramp_speed) final_speed = ramp_speed;
                }
            }
        }

        drive_msg.drive.speed = final_speed;
        drive_pub.publish(drive_msg);

        visualize_trajectories(trajectories, best_idx);
    }

    void visualize_trajectories(const std::vector<std::vector<State>>& trajs, int best_idx) {
        visualization_msgs::MarkerArray markers;
        int step = N_SAMPLES / 20; if (step < 1) step = 1;

        for (int i = 0; i < N_SAMPLES; i += step) {
            visualization_msgs::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = ros::Time::now();
            line.ns = "mpc_candidates";
            line.id = i;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.scale.x = 0.05;
            
            // 撞墙的轨迹画成红色，正常的画灰色，最好的画绿色
            // 这里简单处理：如果 cost 很大(撞墙)，可以画红，但为了简化代码逻辑，只画选中的和未选中的
            if (i == best_idx) {
                line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 1.0;
                line.scale.x = 0.1;
            } else {
                line.color.r = 0.5; line.color.g = 0.5; line.color.b = 0.5; line.color.a = 0.3;
            }
            for (const auto& s : trajs[i]) line.points.push_back(create_point(s.x, s.y));
            markers.markers.push_back(line);
        }
        traj_pub.publish(markers);
    }
    
    geometry_msgs::Point create_point(double x, double y) {
        geometry_msgs::Point p; p.x = x; p.y = y; p.z = 0.1; return p;
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "mpc_follower_node");
    MPCFollower mpc;
    ros::spin();
    return 0;
}