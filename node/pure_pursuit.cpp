#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h> // 可视化
#include <fstream>
#include <vector>
#include <cmath>

struct Point {
    double x;
    double y;
};

class PurePursuit {
private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Publisher marker_pub; // 红球发布者
    ros::Subscriber odom_sub;
    
    std::vector<Point> path_points;
    int last_found_index = 0; // 【核心修改】记住上一次找到的点在哪里

    // 参数调整
    double lookahead_dist = 1.0; 

public:
    PurePursuit() {
        // ⚠️ 请确认路径
        load_path("/home/wangwei/catkin_ws/path.csv"); 

        odom_sub = n.subscribe("/odom", 10, &PurePursuit::odom_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
        marker_pub = n.advertise<visualization_msgs::Marker>("/env_marker", 10);
    }

    void load_path(std::string filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open path file: %s", filename.c_str());
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> seglist;
            while (std::getline(ss, segment, ',')) {
                seglist.push_back(segment);
            }
            if (seglist.size() >= 2) {
                path_points.push_back({std::stod(seglist[0]), std::stod(seglist[1])});
            }
        }
        ROS_INFO("Path loaded: %lu points", path_points.size());
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (path_points.empty()) return;

        double car_x = msg->pose.pose.position.x;
        double car_y = msg->pose.pose.position.y;
        
        // 获取 Yaw
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, car_yaw;
        m.getRPY(roll, pitch, car_yaw);

        // --- 【核心修改：路点追踪算法】 ---
        
        // 步骤 1: 先找到离车最近的那个点 (从上次的位置开始找，防止跳变)
        // 我们只在 last_found_index 附近小范围搜索，优化性能且防止跳到终点
        int closest_index = last_found_index;
        double min_dist_to_car = 10000.0;
        
        // 搜索范围：从上次的位置往后搜 50 个点 (假设 10Hz 录制，车跑不了太远)
        for (int i = 0; i < path_points.size(); i++) {
            // 处理循环索引 (跑完一圈回到 0)
            int idx = (last_found_index + i) % path_points.size();
            
            double d = std::sqrt(pow(path_points[idx].x - car_x, 2) + 
                                 pow(path_points[idx].y - car_y, 2));
            
            if (d < min_dist_to_car) {
                min_dist_to_car = d;
                closest_index = idx;
            }
            
            // 优化：如果距离开始变大了，说明已经过了最近点，可以提前退出
            // (但在闭环处要小心，这里为了稳妥我们搜完全部路径也行，或者搜局部)
        }
        
        // 更新最近点记录
        last_found_index = closest_index;

        // 步骤 2: 从最近点开始，沿着路径往后找，直到距离 >= lookahead_dist
        int target_index = closest_index;
        double dist_to_target = 0.0;
        
        for (int i = 0; i < path_points.size(); i++) {
            int idx = (closest_index + i) % path_points.size();
            double d = std::sqrt(pow(path_points[idx].x - car_x, 2) + 
                                 pow(path_points[idx].y - car_y, 2));
            
            if (d >= lookahead_dist) {
                target_index = idx;
                break; // 找到了！就是这个点
            }
        }
        
        Point best_point = path_points[target_index];

        // --- 坐标变换与控制 ---
        double dx = best_point.x - car_x;
        double dy = best_point.y - car_y;
        double local_y = -sin(car_yaw) * dx + cos(car_yaw) * dy;

        double L2 = lookahead_dist * lookahead_dist;
        double steering_angle = 2 * local_y / L2;

        // 发布控制
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = 1.5; 
        
        if (steering_angle > 0.4) steering_angle = 0.4;
        if (steering_angle < -0.4) steering_angle = -0.4;
        drive_pub.publish(drive_msg);

        // --- 可视化 ---
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = best_point.x;
        marker.pose.position.y = best_point.y;
        marker.pose.position.z = 0.5;
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
        marker_pub.publish(marker);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}