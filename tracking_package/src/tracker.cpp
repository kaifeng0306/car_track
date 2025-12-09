#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "thutracker.hpp"
#include <Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

#include <numeric> 
#include <std_msgs/ColorRGBA.h>
#include "DBSCAN_simple.h"
#include "DBSCAN_precomp.h"
#include "DBSCAN_kdtree.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <cmath>
#include "jsk_recognition_msgs/BoundingBox.h"

using namespace std;

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

Eigen::Matrix4f camera_to_lidar_transform;// 外参矩阵
    // 相机内参矩阵
Eigen::Matrix3f K;

// 全局变量，用于存储当前跟踪的ROI和THUTracker实例
cv::Rect current_roi;
THUTracker tracker;
bool roi_selected = false;
cv::Mat global_rgb;
// 创建一个发布者，用于发布带有跟踪框的图像
ros::Publisher image_pub;
ros::Publisher cloud_pub;
ros::Publisher cloud_pub1;
ros::Publisher path_pub;
ros::Publisher bbox_pub;
ros::Publisher downsample_pub;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 将ROS图像消息转换为OpenCV图像
        cv::Mat frame;
        cv::Mat frame_rgb;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = cv_ptr->image;
        frame_rgb = rgb_ptr->image;
        global_rgb = frame_rgb;

        // 显示图像
        cv::imshow("Display Image", frame_rgb);

        // 检查是否按下了'S'键
        char key = (char)cv::waitKey(10); 
        if (key == 's') {
            // 选择ROI
            current_roi = cv::selectROI("Display Image", frame_rgb, true, false);
            roi_selected = true;
        }

        // 检查ROI是否已经初始化
        if (roi_selected == true) {
            // 选择ROI，这里使用selectROI函数选择感兴趣区域
            // current_roi = cv::selectROI("Display Image", frame, true, false);
            // 初始化tracker
            roi_selected = false;
            tracker.init(current_roi, frame);
        }
        else{
            // 更新tracker
            cv::Rect new_roi = tracker.update(frame);
            current_roi = new_roi;
            // 绘制跟踪框
            cv::rectangle(frame_rgb, new_roi, cv::Scalar(0, 255, 0), 5);
            // 将OpenCV图像转换为ROS图像消息
            sensor_msgs::ImagePtr tracked_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_rgb).toImageMsg();
            // 发布带有跟踪框的图像
            image_pub.publish(tracked_image_msg);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge 转换错误: %s", e.what());
    }
}

void segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
     pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    // 设置过滤条件：z轴值大于等于-1.1
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GE, -0.5)));
    // 创建过滤对象
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    // 设置过滤条件
    condrem.setCondition(range_cond);
    // 过滤点云
    condrem.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    condrem.filter(*filtered_cloud);
    // 将过滤后的点云赋值回输入的cloud指针
    *cloud = *filtered_cloud;
}

double calculateDistance(const geometry_msgs::PoseStamped p1, const geometry_msgs::PoseStamped p2){
    return sqrt((p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x) + (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y));
}

geometry_msgs::PoseStamped lerp(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end, double t) {
    geometry_msgs::PoseStamped result;
    result.header = start.header; // 假设时间戳和框架与起点相同
    result.pose.position.x = start.pose.position.x + t * (end.pose.position.x - start.pose.position.x);
    result.pose.position.y = start.pose.position.y + t * (end.pose.position.y - start.pose.position.y);
    result.pose.position.z = start.pose.position.z; // 假设在二维平面上
    // orientation 可以设置为与起点相同或其他值
    result.pose.orientation = start.pose.orientation;
    return result;
}

std::vector<geometry_msgs::PoseStamped> interpolatePoints(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end) {
    double distance = calculateDistance(start, end);
    int numPoints = static_cast<int>((distance / 0.1) + 1); // 确保包含起点和终点
    std::vector<geometry_msgs::PoseStamped> points;
    
    for (int i = 0; i <= numPoints; ++i) {
        double t = static_cast<double>(i) / numPoints;
        geometry_msgs::PoseStamped pose_stamped = lerp(start, end, t);
        pose_stamped.pose.orientation.w = 1;
        points.push_back(pose_stamped);
    }

    return points;
}

void resetPath(nav_msgs::Path& path)
{
    // 清空poses数组
    path.poses.clear();

    // 设置消息头的时间戳为0（或ros::Time::now()获取当前时间）
    path.header.stamp = ros::Time(0);

    // 设置frame_id为空字符串
    path.header.frame_id = "";

    // 如果需要，可以在这里添加其他重置操作
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将ROS点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // 创建体素网格滤波器对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    // 设置体素网格的大小，这个值可以根据实际情况调整
    vg.setLeafSize(0.02f, 0.02f, 0.02f);

    // 降采样点云
    vg.setInputCloud(pcl_cloud);
    vg.filter(*downsampled_cloud);
    
    segmentPointCloud(downsampled_cloud);
    
    sensor_msgs::PointCloud2Ptr downsample_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*downsampled_cloud, *downsample_msg);
    downsample_msg->header = cloud_msg->header;
    downsample_pub.publish(downsample_msg);


    // 用于存储ROI内点云的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto& point : downsampled_cloud->points) {
        // 将点云点从雷达坐标系转换到相机坐标系
        Eigen::Vector4f lidar_point_4d(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f camera_point_4d = camera_to_lidar_transform * lidar_point_4d;

        // // 投影到图像坐标系
        float x = K(0, 0) * camera_point_4d[0]/camera_point_4d[2] + K(0, 2);
        float y = K(1, 1) * camera_point_4d[1]/camera_point_4d[2] + K(1, 2);
        // float x = -K(0, 0) * camera_point_4d[1]/camera_point_4d[0] + K(0, 2);
        // float y = -K(1, 1) * camera_point_4d[2]/camera_point_4d[0] + K(1, 2);
        
        if (x >= 0 && y >= 0 && x < global_rgb.cols && y < global_rgb.rows) {
            cv::circle(global_rgb, cv::Point(static_cast<int>(x), static_cast<int>(y)), 5, cv::Scalar(0, 255, 0), -1);
        }
        // 检查点是否在ROI内
        if (current_roi.contains(cv::Point(static_cast<int>(x), static_cast<int>(y)))) {
            pcl::PointXYZ roi_point;
            roi_point.x = point.x;
            roi_point.y = point.y;
            roi_point.z = point.z;
            roi_cloud->points.push_back(roi_point);
            cv::circle(global_rgb, cv::Point(static_cast<int>(x), static_cast<int>(y)), 5, cv::Scalar(0, 0, 266), -1);
        }
    }

    // cv::imshow("Projected Points on Live Feed", global_rgb);
    // cv::waitKey(10); 
    
    tree->setInputCloud(roi_cloud);
    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setCorePointMinPts(2);
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(roi_cloud);
    ec.extract(cluster_indices);

    sensor_msgs::PointCloud2Ptr roi_cloud_msg(new sensor_msgs::PointCloud2);
    // 将着色后的点云转换为ROS PointCloud2消息
    pcl::toROSMsg(*roi_cloud, *roi_cloud_msg);
    roi_cloud_msg->header = cloud_msg->header;

    // 发布着色后的点云消息
    cloud_pub.publish(roi_cloud_msg);

    geometry_msgs::Point end_point;
    nav_msgs::Path global_path;

    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        const pcl::PointIndices &cluster = cluster_indices[i];
        float total_depth = 0.0f;
        size_t point_count = 0;
        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

        for (size_t j = 0; j < cluster.indices.size(); ++j) {
            int point_index = cluster.indices[j];
            const pcl::PointXYZ &point = roi_cloud->points[point_index];
            // 计算每个点的深度：sqrt(x^2 + y^2 + z^2)
            float depth = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            total_depth += depth;
            
            // 累加点的坐标值
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            
            ++point_count;
        }
        
        // 计算聚类的平均深度
        if (point_count > 0) {
            float avg_depth = total_depth / point_count;
            std::cout << "Average depth of cluster " << i << " is: " << avg_depth << std::endl;
        }
        
        // 计算聚类的中心点坐标
        if (point_count > 0 && i == 0) {
            end_point.x = sum_x / point_count;
            end_point.y = sum_y / point_count;
            //end_point.z = sum_z / point_count;
            end_point.z = 0 - 0.7;
            jsk_recognition_msgs::BoundingBox bbox;
            // bbox.header.frame_id = "rslidar";
            bbox.header.frame_id = "rslidar";
            bbox.pose.position.x = end_point.x;
            bbox.pose.position.y = end_point.y;
            bbox.pose.position.z = sum_z / point_count;
            bbox.dimensions.x = 0.7;
            bbox.dimensions.y = 0.7;
            bbox.dimensions.z = 0.7;
            bbox_pub.publish(bbox);
        }
    }

    geometry_msgs::PoseStamped start_pose, end_pose;
    start_pose.pose.position.x = 0 - 0.3; // 起点x坐标，车体中心
    start_pose.pose.position.y = 0;
    start_pose.pose.position.z = 0-0.7;
    start_pose.pose.orientation.w = 1;
    // start_pose.header.frame_id = "rslidar";
    start_pose.header.frame_id = "rslidar";

    end_pose.pose.position = end_point;
    end_pose.pose.orientation.w = 1;
    // end_pose.header.frame_id = "rslidar";
    end_pose.header.frame_id = "rslidar";
    std::vector<geometry_msgs::PoseStamped> points = interpolatePoints(start_pose, end_pose);
    
   // global_path.header.frame_id = "rslidar";
   global_path.header.frame_id = "rslidar";
    // 将起点和终点添加到路径中
    for(int i=0; i < points.size(); i++){
        global_path.poses.push_back(points[i]);   
    }
    // 发布路径
    
    path_pub.publish(global_path);
    resetPath(global_path);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    
    // real world
    float elements[] = {
        -0.0362488, -0.99919, -0.0131462, -0.112901,
        -0.0562503, 0.0151798, -0.998297, 0.33908,
        0.997749, -0.0354419, -0.0567576, 0.260649,
        0, 0, 0, 1
    };
    
    // simulation
    // float elements[] = {
    //     1.0, 0.0, 0.0, 0.0,
    //     0.0, 1.0, 0.0, 0.0,
    //     0.0, 0.0, 1.0, 0.1,
    //     0.0, 0.0, 0.0, 1.0
    // };
    
    camera_to_lidar_transform << elements[0], elements[1], elements[2], elements[3],
                             elements[4], elements[5], elements[6], elements[7],
                             elements[8], elements[9], elements[10], elements[11],
                             elements[12], elements[13], elements[14], elements[15];

    // real world    
    K << 1485.014632, 0.000000, 1524.241807,
            0.000000, 1482.979511, 1021.861100,
            0.000000, 0.000000, 1.000000;
    
    // simulation
    // K << 762.7249337622711, 0.0, 640.5, 
    //             0.0, 762.7249337622711, 360.5, 
    //             0.0, 0.0, 1.0;
    
    // 检查是否提供了图像话题的参数
    std::string image_topic;
    nh.param<std::string>("image_topic", image_topic, "/hikrobot_camera/rgb");

    // real world
    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, imageCallback);
    ros::Subscriber cloud_sub = nh.subscribe("/rslidar_points", 1, cloudCallback);

    //simulation
    // ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 1, imageCallback);
    // ros::Subscriber cloud_sub = nh.subscribe("/velodyne_points", 1, cloudCallback);

    // 创建一个发布者，发布跟踪图像话题
    image_pub = nh.advertise<sensor_msgs::Image>("tracked_image", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("roi_cloud", 1);
    cloud_pub1 = nh.advertise<sensor_msgs::PointCloud2>("transform", 1);
    path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);
    bbox_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>("bbox", 10);
    downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("downsample_cloud", 1);

    // 创建并初始化OpenCV窗口来显示图像
    namedWindow("Display Image", cv::WINDOW_NORMAL);

    // 进入ROS循环
    ros::spin();

    return 0;
}
