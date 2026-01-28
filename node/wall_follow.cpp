#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <cmath>

class WallFollower {
private:
    ros::NodeHandle n;
    ros::Publisher drive_pub;
    ros::Subscriber scan_sub;

    // PID 参数 (面试时常问：怎么调参？)
    // KP: 比例系数，负责"修正现在的误差"
    // KD: 微分系数，负责"阻尼震荡，预测未来"
    double kp = 1.0; 
    double kd = 0.1; 
    double prev_error = 0.0; // 上一次的误差
    double distance_to_keep = 1.0; // 目标离墙距离 (米)

public:
    WallFollower() {
        n = ros::NodeHandle();
        
        // 1. 订阅激光雷达数据 /scan
        scan_sub = n.subscribe("/scan", 10, &WallFollower::scan_callback, this);
        
        // 2. 发布控制指令到 /nav 话题
        // 注意：这里发布到 /nav，需要在仿真器里按 'n' 键激活
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // --- 感知部分 ---
        
        // 我们想找车身左侧 (90度方向) 的激光雷达数据
        // 雷达扫描范围通常是 -135度 到 +135度 (弧度制)
        // scan_msg->angle_min 是起始角度
        // scan_msg->angle_increment 是每个激光点的间隔
        
        // 计算左侧90度对应的索引
        double angle_left = 1.57; // 90度约为 1.57 弧度
        int index_left = (angle_left - scan_msg->angle_min) / scan_msg->angle_increment;

        // 获取距离 (注意要检查索引是否越界，这里简单处理)
        if (index_left < 0 || index_left >= scan_msg->ranges.size()) return;
        
        double current_dist = scan_msg->ranges[index_left];
        
        // 简单的滤波：如果距离无穷大或为0（通常是数据错误），就忽略
        if (!std::isfinite(current_dist) || current_dist == 0.0) return;

        // --- 计算 PID ---
        
        // 误差 = 目标距离 - 实际距离
        // 如果 error > 0，说明离墙太远 (current_dist < target)，需要向右转吗？
        // 不，如果我们在左侧沿墙：
        // 离墙太远 (current > target) -> error < 0 -> 应该左转靠近墙
        double error = distance_to_keep - current_dist;
        
        // 舵机转向值 = P控制 + D控制
        double steering_angle = -(kp * error + kd * (error - prev_error));
        
        // 更新上一次误差
        prev_error = error;

        // 限制最大转向角 (防止舵机打死，仿真器最大约0.4弧度)
        if (steering_angle > 0.4) steering_angle = 0.4;
        if (steering_angle < -0.4) steering_angle = -0.4;

        // --- 控制部分 ---
        
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.drive.speed = 1.5; // 恒定速度 1.5 m/s
        drive_msg.drive.steering_angle = steering_angle;
        
        drive_pub.publish(drive_msg);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wall_follower_node");
    WallFollower wf;
    ros::spin();
    return 0;
}