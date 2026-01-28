#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <vector>
#include <cmath>

// 这是一个简单的路径录制节点
// 它会把你的行车轨迹保存到 /home/你的用户名/rcws/logs/path.csv (请修改路径)

class WaypointLogger {
private:
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    std::ofstream outfile;
    double last_x = 0.0;
    double last_y = 0.0;

public:
    WaypointLogger() {
        // 请把下面的路径改成你电脑上实际想保存的路径！
        // 比如 "/home/wangwei/catkin_ws/path.csv"
        outfile.open("/home/wangwei/catkin_ws/path.csv"); 

        // 订阅里程计 /odom
        odom_sub = n.subscribe("/odom", 10, &WaypointLogger::odom_callback, this);
    }

    ~WaypointLogger() {
        outfile.close();
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        
        // 为了避免保存太密集的点，只有当移动超过 0.1 米才记录
        double dist = std::sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
        if (dist > 0.1) {
            // 格式: x, y, 速度(假设恒定)
            outfile << x << "," << y << "," << "1.5" << std::endl;
            last_x = x;
            last_y = y;
            ROS_INFO("Recorded: %.2f, %.2f", x, y);
        }
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "waypoint_logger");
    WaypointLogger logger;
    ros::spin();
    return 0;
}