#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <iostream>
#include <radar/radar_data.h>

// 计算校验和
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum += data[i];
    }
    return checksum & 0xFF;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_communication_node");
    ros::NodeHandle nh;

    serial::Serial ser;
    std::string port = "/dev/ttyACM0"; // 串口设备路径，请根据实际情况修改
    uint32_t baudrate = 9600; // 波特率，请根据实际情况修改

    // 创建消息类型的对象
    radar::radar_data radar_msg; // 替换为实际的消息类型

    // 创建发布者
    ros::Publisher radar_pub = nh.advertise<radar::radar_data>("radar_data", 10); 

    try {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port " << port);
        return -1;
    }

    if (!ser.isOpen()) {
        ROS_ERROR_STREAM("Serial port not open");
        return -1;
    }

    ROS_INFO_STREAM("Serial port initialized");

    ros::Rate loop_rate(10); // 设置发送和接收频率为10Hz

    while (ros::ok()) {
        // 发送指令使四个雷达同时测距
        uint8_t command[] = {0x55, 0xAA, 0x01, 0x01, 0x01};
        ser.write(command, sizeof(command));

        // 等待一段时间确保雷达有足够的时间测距并发送回数据
        ros::Duration(0.1).sleep();

        // 接收测距结果
        if (ser.available() >= 12) { // 包括校验和的13个字节
             uint8_t res[13];
            int bytesRead = 0;
	    bytesRead = ser.read(res, sizeof(res));
            ROS_INFO_STREAM("bytesRead" << bytesRead);
            ROS_INFO_STREAM("buffer" << res);
            // 验证校验和
            uint8_t checksum = calculateChecksum(res, 12);
            if (checksum == res[12]) {
                // 填充消息对象
                radar_msg.distance_1 = (res[4] << 8) | res[5];
                radar_msg.distance_2 = (res[6] << 8) | res[7];
                radar_msg.distance_3 = (res[8] << 8) | res[9];
                radar_msg.distance_4 = (res[10] << 8) | res[11];

                // 发布消息
                radar_pub.publish(radar_msg);

                // 输出调试信息
                ROS_INFO("Published radar data.");
                ROS_INFO("Distance 1: %u mm", radar_msg.distance_1);
                ROS_INFO("Distance 2: %u mm", radar_msg.distance_2);
                ROS_INFO("Distance 3: %u mm", radar_msg.distance_3);
                ROS_INFO("Distance 4: %u mm", radar_msg.distance_4);
            } else {
                ROS_WARN("Checksum mismatch!");
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

