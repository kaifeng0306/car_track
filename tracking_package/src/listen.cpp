#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <iostream>

// 回调函数不是必需的，因为我们将手动获取变换
// void callback(const tf::TransformStampedConstPtr& msg) {
//     // 这个回调函数将会在每次接收到/tf消息时被调用
//     // 但在这个例子中我们不会使用它
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle nh;

    // 创建一个Transform Listener对象
    tf::TransformListener listener;

    // 订阅/tf话题
    // 注意：这里我们实际上并不需要订阅/tf话题来接收所有tf消息
    // 因为我们将使用Transform Listener来获取特定的变换
    // 但是如果你想要根据接收到的消息做一些事情，你可以这样订阅
    // ros::Subscriber sub = nh.subscribe("tf", 100, callback);

    try {
        // 等待所需的坐标帧变换
        listener.waitForTransform("camera_link", "velodyne_base_link", ros::Time(0), ros::Duration(3.0));

        // 查找从velodyne_base_link到camera_link的变换
        tf::StampedTransform transform;
        listener.lookupTransform("camera_link", "velodyne_base_link", ros::Time(0), transform);

        // 转换为Eigen矩阵
        Eigen::Matrix4d eigen_matrix;
        eigen_matrix.setIdentity();
        eigen_matrix.block<3, 3>(0, 0) = tf::Matrix3x3(transform.getRotation()).cast<double>();
        eigen_matrix.block<3, 1>(0, 3) = transform.getOrigin().x(),
        eigen_matrix(1, 3) = transform.getOrigin().y();
        eigen_matrix(2, 3) = transform.getOrigin().z();

        // 输出外参矩阵
        std::cout << "Extrinsic Matrix:\n" << eigen_matrix << std::endl;
    } catch (const tf::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return -1;
    }

    // 进入ROS循环，保持节点运行
    ros::spin();

    return 0;
}