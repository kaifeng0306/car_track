#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "velocity_publisher");

    // 创建一个节点句柄
    ros::NodeHandle n;

    // 创建一个发布者，发布到/cmd_vel主题，消息类型为geometry_msgs/Twist
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // 循环频率设置为10Hz
    ros::Rate loop_rate(10);

    // 循环100次
    for (int i = 0; i < 500; ++i)
    {
        // 创建一个geometry_msgs/Twist消息
        geometry_msgs::Twist twist;

        // 设置线性速度为0，角速度z轴为0.5
        twist.linear.x = 0.1;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        // 发布消息
        cmd_vel_pub.publish(twist);

        // 等待0.1秒
        ros::Duration(0.1).sleep();
        loop_rate.sleep();
    }

    // 发布停止命令
    geometry_msgs::Twist stop_twist;
    stop_twist.linear.x = 0.0;
    stop_twist.linear.y = 0.0;
    stop_twist.linear.z = 0.0;
    stop_twist.angular.x = 0.0;
    stop_twist.angular.y = 0.0;
    stop_twist.angular.z = 0.0;
    cmd_vel_pub.publish(stop_twist);

    return 0;
}
