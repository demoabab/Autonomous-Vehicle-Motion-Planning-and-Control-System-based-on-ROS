#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cmath>

struct Point {
    double x;
    double y;
};

class RRTFollower {
private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber path_sub;
    
    std::vector<Point> path_points;
    int last_found_index = 0; 
    
    // 关键参数
    double lookahead_dist = 1.0;  // 前视距离
    double forward_speed = 1.5;   // 前进速度
    double reverse_speed = -1.0;  // 倒车速度 (负数，且慢一点)

public:
    RRTFollower() {
        // 订阅 Hybrid A* 发出来的路径
        path_sub = n.subscribe("/planned_path", 1, &RRTFollower::path_cb, this);

        odom_sub = n.subscribe("/odom", 10, &RRTFollower::odom_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
        marker_pub = n.advertise<visualization_msgs::Marker>("/follower_debug", 10);
        
        ROS_INFO("Hybrid Follower initialized. Waiting for /planned_path...");
    }

    void path_cb(const nav_msgs::Path::ConstPtr& msg) {
        path_points.clear();
        for (const auto& pose : msg->poses) {
            path_points.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        // 新路径来了，重置索引，从头开始跑
        last_found_index = 0; 
        ROS_INFO("Received new path with %lu points.", path_points.size());
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (path_points.empty()) return;

        double car_x = msg->pose.pose.position.x;
        double car_y = msg->pose.pose.position.y;
        
        // 获取 Yaw 角
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, car_yaw;
        m.getRPY(roll, pitch, car_yaw);

        // --- 1. 寻找最近的路点 (Search Window) ---
        int closest_index = last_found_index;
        double min_dist_to_car = 10000.0;
        
        // 简单优化：只在当前索引后面搜寻，防止回跳
        // 如果是 Hybrid A* 这种可能掉头的路径，搜索范围稍微大一点比较安全
        for (int i = 0; i < path_points.size(); i++) {
            // 这里我们遍历整个路径，因为掉头时路径可能会折叠，为了稳妥
            // 实际工程中应该用滑动窗口
            double d = std::sqrt(pow(path_points[i].x - car_x, 2) + 
                                 pow(path_points[i].y - car_y, 2));
            
            if (d < min_dist_to_car) {
                min_dist_to_car = d;
                closest_index = i;
            }
        }
        last_found_index = closest_index;

        // --- 2. 寻找目标点 (Lookahead) ---
        // 从最近点开始，沿着路径找 1 米远的点
        int target_index = closest_index;
        for (int i = closest_index; i < path_points.size(); i++) {
            double d = std::sqrt(pow(path_points[i].x - car_x, 2) + 
                                 pow(path_points[i].y - car_y, 2));
            if (d >= lookahead_dist) {
                target_index = i;
                break;
            }
        }
        // 如果跑到了尽头，就选最后一个点
        if (target_index >= path_points.size()) target_index = path_points.size() - 1;
        
        Point best_point = path_points[target_index];

        // --- 3. 坐标变换 (Global -> Vehicle) ---
        double dx = best_point.x - car_x;
        double dy = best_point.y - car_y;
        
        // 旋转矩阵
        double local_x = cos(-car_yaw) * dx - sin(-car_yaw) * dy;
        double local_y = -sin(car_yaw) * dx + cos(car_yaw) * dy;

	// --- 4. 智能换挡与动态变速 ---
        double L2 = lookahead_dist * lookahead_dist;
        double steering_cmd = 2 * local_y / L2;
        double speed_cmd = 0.0;

        // 定义赛车参数
        double max_speed = 5;  // 尝试 3.5 m/s (你可以试着改到 5.0 挑战极限)
        double min_speed = 1.2;  // 过弯保底速度
        double speed_gain = 4.0; // 转向减速系数

        if (local_x >= 0) {
            // 前进：动态计算速度
            speed_cmd = max_speed - speed_gain * std::abs(steering_cmd);
            if (speed_cmd < min_speed) speed_cmd = min_speed;
        } else {
            // 倒车：恒定慢速
            speed_cmd = reverse_speed; 
        }

        // --- 5. 停车逻辑 (线性减速) ---
        double dist_to_final = std::sqrt(pow(path_points.back().x - car_x, 2) + 
                                         pow(path_points.back().y - car_y, 2));
        double stopping_threshold = 0.1; 
        double slow_down_dist = 1.0;

        if (dist_to_final < stopping_threshold) {
            speed_cmd = 0.0;
        } else if (dist_to_final < slow_down_dist) {
            double slow_speed = (dist_to_final / slow_down_dist) * std::abs(speed_cmd);
            if (slow_speed < 0.3) slow_speed = 0.3; 
            if (speed_cmd > 0) speed_cmd = slow_speed;
            else speed_cmd = -slow_speed;
        }

        // --- 6. 发布指令 ---
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.drive.steering_angle = steering_cmd;
        drive_msg.drive.speed = speed_cmd;
        
        // 限幅
        if (drive_msg.drive.steering_angle > 0.4) drive_msg.drive.steering_angle = 0.4;
        if (drive_msg.drive.steering_angle < -0.4) drive_msg.drive.steering_angle = -0.4;

        drive_pub.publish(drive_msg);

        // --- 7. 可视化 (蓝球) ---
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "follower";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = best_point.x;
        marker.pose.position.y = best_point.y;
        marker.pose.position.z = 0.5;
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3;
        // 颜色根据档位变化：前进蓝色，倒车红色
        marker.color.a = 1.0; 
        if (speed_cmd >= 0) {
             marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; // 蓝
        } else {
             marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // 红
        }
        marker_pub.publish(marker);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "hybrid_follower_node");
    RRTFollower follower;
    ros::spin();
    return 0;
}