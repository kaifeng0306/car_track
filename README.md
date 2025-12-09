# 🚗👣 Unmanned Vehicle Human-Following System

本项目实现了基于视觉与激光雷达的 **无人车人车跟随系统**。系统利用 **KCF+FDSST 图像跟踪算法** 与 **激光雷达点云聚类** 技术获取目标位置，并结合 **Pure Pursuit (PP) 路径跟随算法** 控制无人车，实现对目标人物的实时跟随。

---

## 📖 功能特性
- **目标检测与跟踪**：基于 KCF + FDSST 算法，支持多尺度特征提取与实时更新。
- **激光雷达融合**：通过相机与 LiDAR 外参标定，将 ROI 区域的点云映射到图像平面，利用 DBSCAN 聚类计算目标距离与三维位置。
- **路径跟随控制**：采用 Pure Pursuit 算法计算无人车线速度与角速度，保证平稳跟随。
- **ROS 可视化支持**：在 RViz 中实时显示车体范围、目标 ROI、路径曲线和控制效果。
- **紧急制动机制**：结合超声波雷达实现前方障碍物检测与紧急停车。

---

## 🛠 系统架构

┌─────────────────────┐
│ Tracking Package │ ← 摄像头输入 (KCF+FDSST)
└─────────┬───────────┘
				      │
                                      ▼
┌─────────────────────┐
│ Lidar Processing │ ← 激光雷达输入 + ROI 点云聚类
└─────────┬───────────┘
                                      │
                                      ▼
┌─────────────────────┐
│ Pure Pursuit (PP) │ ← 路径点生成与运动控制
└─────────┬───────────┘
                                      │
                                      ▼
┌─────────────────────┐
│ Base Controller │ ← 串口控制差速底盘
└─────────────────────┘


---

## 📦 环境依赖
- **系统环境**：Ubuntu 18.04 + ROS Melodic  
- **硬件设备**：
  - 海康工业相机 MV-CS060-10UC-PRO  
  - 速腾 M1 固态激光雷达  
  - 超声波雷达
  - 差速底盘车体
- **软件依赖**：
  - OpenCV 3.4.1
  - ROS Packages: `tracking_package`, `radar`, `ros_four1_msg`, `car_display`  
  - DBSCAN 聚类算法实现  

---

## ⚙️ 安装与配置
1. 克隆本仓库到 ROS 工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/yourname/human-following-car.git
   cd ..
   catkin_make

安装依赖：
sudo apt-get install ros-melodic-rviz ros-melodic-tf
sudo apt-get install libopencv-dev

硬件串口权限设置（具体串口名称在相关代码中根据具体名称进行修改）：
sudo chmod 777 /dev/ttyUSB0   # 底盘（ros_four1_msg/src/ros_four_msg.cpp第133行ser.setPort("/dev/ttyUSB0");）
sudo chmod 777 /dev/ttyACM0   # 超声波雷达（radar/src/radar.cpp第21行std::string port = "/dev/ttyACM0"; // 串口设备路径，请根据实际情况修改）

---

## 🚀 使用方法

启动图像跟踪 (调试用，不涉及底盘运动)：
  ```bash
  source devel/setup.bash
  roslaunch tracking_package tracker.launch

按 s 键选择跟踪目标

按 空格/回车 开始跟踪

启动车辆跟随 (实际运行)：

roslaunch tracking_package start.launch

RViz 可视化：

目标 ROI、车体模型、路径曲线将实时显示

绿线：跟随路径

红点：路径最近点

绿色框：目标物体的 3D bounding box


```mermaid
flowchart TD
    A[Tracking Package<br>(摄像头输入: KCF+FDSST)] --> B[Lidar Processing<br>(激光雷达输入 + ROI 点云聚类)]
    B --> C[Pure Pursuit (PP)<br>(路径点生成与运动控制)]
    C --> D[Base Controller<br>(串口控制差速底盘)]

  ```

