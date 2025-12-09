#include <ros_four1_msg.h>
#include <ros/ros.h>                      //类似 C 语言的 stdio.h 
#include <ros_four1_msg/four1.h>            //要用到 msg 中定义的数据类型 
#include <ros_four1_msg/radar_data.h>
#include <serial/serial.h>                //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <signal.h>


#define Base_Width 730  //轴距

serial::Serial ser; //声明串口对象 
const uint16_t threshold = 500; // 根据实际情况设置阈值
bool flag;
int state;
/*************************************************************************/
static void open20MsData(u8 data)
{
  Open20MsData.prot.Header = HEADER;
  Open20MsData.prot.Len    = 10;
  Open20MsData.prot.Type   = 4;
  Open20MsData.prot.Cmd    = 0x01;
  Open20MsData.prot.Num    = 1;
  Open20MsData.prot.Data   = data;
  Open20MsData.prot.Check  = 0;
  for(int i=0;i < Open20MsData.prot.Len - 2;i++)
  {
    Open20MsData.prot.Check += Open20MsData.data[i];
  }
  ser.write(Open20MsData.data,sizeof(Open20MsData.data));
}

/*************************************************************************/
//速度设置，Vx线速度;Vz角速度;StopCon急停设置，0不急停，1急停
void setSpeed(float Vx,float Vz,u8 StopCon)
{
  s16 TempLSpeed=0,TempRSpeed=0;

  TempLSpeed = Vx*1000 - Vz*Base_Width/2.0;
  TempRSpeed = Vx*1000 + Vz*Base_Width/2.0;

  TXRobotData1.prot.Header  = HEADER;
  TXRobotData1.prot.Len     = 16;
  TXRobotData1.prot.Type    = 4;
  TXRobotData1.prot.Cmd     = 0x02;
  TXRobotData1.prot.Num     = 4;
  TXRobotData1.prot.FLSpeed = TempLSpeed;
  TXRobotData1.prot.FRSpeed = TempRSpeed;
  TXRobotData1.prot.BLSpeed = TempLSpeed;
  TXRobotData1.prot.BRSpeed = TempRSpeed;
  TXRobotData1.prot.StopCon = StopCon;
  TXRobotData1.prot.Check   = 0;

  for(int i=0;i < sizeof(TXRobotData1.data) - 2;i++)
  {
    TXRobotData1.prot.Check += TXRobotData1.data[i];
  }
  ser.write(TXRobotData1.data,sizeof(TXRobotData1.data));
  
}

void radarCallback(const ros_four1_msg::radar_data& radar_msg)
{
    std::cout<<"radar"<<std::endl;
    // 检查是否有距离小于阈值
    bool isObstacleClose = (radar_msg.distance_1 > 0 && radar_msg.distance_1 < threshold) || (radar_msg.distance_2 > 0 && radar_msg.distance_2 < threshold) || (radar_msg.distance_3 > 0 && radar_msg.distance_3 < threshold) || (radar_msg.distance_4 > 0 && radar_msg.distance_4 < threshold);
	
    // 如果有障碍物距离过近，则执行急停
    if (isObstacleClose)
    {	
	flag = true;
	state = 1;
std::cout<<"ji ting"<<std::endl;
    }
    else if(((radar_msg.distance_1 == 0 && radar_msg.distance_1 > threshold) || (radar_msg.distance_2 == 0 && radar_msg.distance_2 > threshold) || (radar_msg.distance_3 == 0 && radar_msg.distance_3 > threshold) || (radar_msg.distance_4 == 0 && radar_msg.distance_4 > threshold)
) && state == 1)
    {
        flag = false;
	state = 0;
    }
}

/*************************************************************************/
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
   if(flag && state == 0){
      setSpeed(0,0,1);
      state = 1;	
   }
   else if (state == 0)
   setSpeed(twist_aux.linear.x,twist_aux.angular.z,0);
}
/*************************************************************************/
void twist_cmd_velCallback(const geometry_msgs::TwistStamped &twist_aux)//速度控制回调
{
   setSpeed(twist_aux.twist.linear.x,twist_aux.twist.angular.z,0);
}
/*************************************************************************/
//当关闭包时调用，关闭20ms上传
void mySigIntHandler(int sig)
{
   ROS_INFO("close the serial!\n");
   open20MsData(0);
   //ser.close();
   ros::shutdown();
}
/*************************************************************************/
int main(int argc,char **argv)
{    
   u16 len=0,TempCheck=0;
   u8  data[200];
   u16 TimeCnt=0;

   ros::init(argc,argv,"ros_four1_talker",ros::init_options::NoSigintHandler);            //解析参数，命名节点为 talker
   signal(SIGINT, mySigIntHandler);  										//把信号槽连接到mySigIntHandler保证关闭节点时能够关闭20ms数据上传

   ros_four1_msg::four1 four_msg;
   ros::NodeHandle nh;                       //创建句柄，相当于一套工具，可以实例化 node，并且对 node 进行操作
   ros::Publisher pub = nh.advertise<ros_four1_msg::four1>("four_info",1);//创建 publisher 对象

   ros::Subscriber twist_cmd_sub = nh.subscribe("/twist_cmd", 100, twist_cmd_velCallback);//速度回调
   ros::Subscriber sub = nh.subscribe("/cmd_vel",100, cmd_velCallback);  //
   ros::Subscriber radar_sub = nh.subscribe("/radar_data", 10, radarCallback);

   try  
    { 
         //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(2000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

   //开启20ms上传
   open20MsData(1);
                              
   while(ros::ok())
   {                                                                 
      //wait have date to spinince ,the callback
      ser.read(RXRobotData20MS.data,sizeof(RXRobotData20MS.data));
      //get check
      TempCheck = 0;
      for(u8 i=0;i<sizeof(RXRobotData20MS.data)-2;i++)
      {
         TempCheck += RXRobotData20MS.data[i];
      }

      //头和校验正确
      if(RXRobotData20MS.prot.Header == HEADER && RXRobotData20MS.prot.Check == TempCheck)
      {

         //消息赋值
         four_msg.FLSpeed = RXRobotData20MS.prot.FLSpeed; //前左轮速度
         four_msg.FRSpeed = RXRobotData20MS.prot.FRSpeed; //前右轮速度
         four_msg.BLSpeed = RXRobotData20MS.prot.BLSpeed; //后左轮速度
         four_msg.BRSpeed = RXRobotData20MS.prot.BRSpeed; //后右轮速度
         four_msg.FLAddEN = RXRobotData20MS.prot.FLAddEN; //前左轮编码器增量
         four_msg.FRAddEN = RXRobotData20MS.prot.FRAddEN; //前右轮编码器增量
         four_msg.BLAddEN = RXRobotData20MS.prot.BLAddEN; //后左轮编码器增量
         four_msg.BRAddEN = RXRobotData20MS.prot.BRAddEN; //后右轮编码器增量
         four_msg.Voltage = RXRobotData20MS.prot.Voltage; //电压×10
         four_msg.State   = RXRobotData20MS.prot.State;   //地盘状态
         pub.publish(four_msg);                      //发布消息
/*
         printf("FL:%d FR:%d BL:%d BR:%d FLA:%d FRA:%d BLA:%d BRA:%d POW:%d St:%d\n",
            RXRobotData20MS.prot.FLSpeed,   RXRobotData20MS.prot.FRSpeed,
            RXRobotData20MS.prot.BLSpeed,   RXRobotData20MS.prot.BRSpeed, 
            RXRobotData20MS.prot.FLAddEN,   RXRobotData20MS.prot.FRAddEN,   
            RXRobotData20MS.prot.BLAddEN,   RXRobotData20MS.prot.BRAddEN,
            RXRobotData20MS.prot.Voltage,   RXRobotData20MS.prot.State);
          
         printf("------------------------------------------------\n");*/
         memset(RXRobotData20MS.data,0,sizeof(RXRobotData20MS.data));
      
      }
      else     //
      {
         printf("Send The ON 20ms up date CMD %x %x %x\n",RXRobotData20MS.prot.Header,RXRobotData20MS.prot.Check,TempCheck);

         len = ser.available();
         //清空数据残余
         if(len > 0)
         {
            ser.read(data,len);
         }
         open20MsData(1);
      
      }
      ros::spinOnce();
   }
   return 0;
}
/*************************************************************************/
