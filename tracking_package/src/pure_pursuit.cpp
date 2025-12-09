#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

ros::Publisher twist_pub;
ros::Publisher visualization_pub;

// 计算两点之间的角度，并判断是左转还是右转
double calculateAngleAndDirection(double x1, double y1, double x2, double y2) {
    // 计算两点之间的差值
    double dx = x2 - x1;
    double dy = y2 - y1;

    // 使用atan2来计算角度，它返回的是弧度值
    double angle_rad = atan2(abs(dx), abs(dy));

    // 判断是左转还是右转
    if (dx > 0) {
        return -(angle_rad); 
    } else if (dx < 0) {
        return (angle_rad); 
    } else {
        return 0.0;
    }
}

double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) +
                     (p2.y - p1.y) * (p2.y - p1.y) +
                     (p2.z - p1.z) * (p2.z - p1.z));
}

int findClosestPose(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& start_pose, double lookahead) {
    double min_distance = std::numeric_limits<double>::max();
    if(lookahead == 0)
    return 0;
    int closest_index = -1;
    for (size_t i = 1; i < path.poses.size(); ++i) {
        double distance = calculateDistance(start_pose.pose.position, path.poses[i].pose.position);
        if (abs(distance - lookahead) <= 0.1 && abs(distance - lookahead) <= min_distance) {
            min_distance = abs(distance - lookahead);
            closest_index = i;
        }
    }
    return closest_index;
}

// 可视化最近的点
void visualizeClosestPoint(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& start_pose, double lookahead) {
    int closest_index = findClosestPose(path, start_pose, lookahead);
    if (closest_index != -1) {
        // 创建Marker消息
        visualization_msgs::Marker marker;
        marker.header = path.header; // 使用路径的头信息
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE; // 可视化为球体
        marker.pose = path.poses[closest_index].pose; // 设置为最近的位姿
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2; // 尺寸
        marker.color.a = 1.0; // 完全不透明
        marker.color.r = 255; // 红色
        marker.color.g = 0.0; // 绿色
        marker.color.b = 0.0; // 蓝色

        // 发布Marker
        visualization_pub.publish(marker);
    }
}

// 回调函数，用于接收Path消息
void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    int number = msg->poses.size();
    const nav_msgs::Path& path = *msg; 
    std::cout << "size" << number << std::endl;
    if(msg->poses.size() > 2){
        std::cout << "speed" << std::endl;
        // 提取第一个点和第二个点，第一个点为车的位置，在baselink中固定为(0,0)，第二个为最近的点，在pushback的时候按照插值顺序pushback的，能保证是最近的点。
        geometry_msgs::PoseStamped start_pose = msg->poses[0];
        geometry_msgs::PoseStamped end_pose = msg->poses[1];
        geometry_msgs::PoseStamped dis_pose = msg->poses[msg->poses.size() - 1];

        double linear_velocity = 0; // 线速度
        double look_ahead = 0;
        double kp = 0;
        // 计算两点之间的直线距离
        double dx = dis_pose.pose.position.x - start_pose.pose.position.x;
        double dy = dis_pose.pose.position.y - start_pose.pose.position.y;
        double dz = dis_pose.pose.position.z - start_pose.pose.position.z; // 其实可以不要，z值固定为0
        double distance = sqrt(dx * dx + dy * dy + dz * dz);
        ROS_INFO("distance: %.4f", distance);
        if (distance <= 3){
            linear_velocity = 0;
            look_ahead = 0;
            kp = 0;
        }
        else if(distance <= 5){
            look_ahead = 1;
            linear_velocity = 0.2;
            kp = 0.01;
        }
        else if(distance <= 8){
            look_ahead = 1.5;
            linear_velocity = 0.2;
            kp = 0.1;
        }
        else{
            look_ahead = 2;
            linear_velocity = 0.2;
            kp = 0.5;
        }
        double angle_rad = calculateAngleAndDirection(start_pose.pose.position.y,  start_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.x);
        double degrees = angle_rad * (180.0 / M_PI);

        look_ahead += kp * (linear_velocity);
        visualizeClosestPoint(path, start_pose, look_ahead);
        double angular_velocity = 0;
        if(look_ahead != 0)
            angular_velocity = 2 * linear_velocity * sin(angle_rad) / look_ahead;

        // real world
        if (degrees < 0) {
            angular_velocity = std::abs(angular_velocity);
        } else {
            angular_velocity = -std::copysign(std::abs(angular_velocity), degrees);
        }

        //simulation
        // if (degrees < 0) {
        //     angular_velocity = -std::copysign(std::abs(angular_velocity), degrees);
        // } else {
        //     angular_velocity = -std::abs(angular_velocity);
        // }
        ROS_INFO("degrees: %.2f ", degrees);
        ROS_INFO("steer_angle: %.2f ", angular_velocity);

        geometry_msgs::Twist twist;
        twist.linear.x = linear_velocity;  
        twist.angular.z = angular_velocity;  
        twist_pub.publish(twist);

        // 输出结果
        ROS_INFO("Linear velocity: %.2f m/s", linear_velocity);
        ROS_INFO("Angular velocity: %.2f rad/s", angular_velocity);
    }
    else{
        geometry_msgs::Twist twist;
        twist.linear.x = 0;  
        twist.angular.z = 0;  
        twist_pub.publish(twist);
    }
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pp_node");
    ros::NodeHandle nh;

    // 创建订阅者，订阅Path消息
    ros::Subscriber path_sub = nh.subscribe("/planned_path", 1000, pathCallback);
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    visualization_pub = nh.advertise<visualization_msgs::Marker>("Marker", 1);

    // 进入ROS循环
    ros::spin();

    return 0;
}
