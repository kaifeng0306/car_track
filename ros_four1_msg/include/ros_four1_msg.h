#ifndef __ROA_FOUR_MSG_H_
#define __ROA_FOUR_MSG_H_
#include <ros/ros.h>    

/*************************************************************************/
#define HEADER    0xDEED		//数据头

#define SWAP16(X)  (((X & 0XFF00) >> 8) | ((X & 0XFF) << 8))
#define SWAP32(X)  ((X & 0XFF) << 24 | (X & 0XFF00) << 8 | (X & 0XFF0000) >> 8 | (X >> 24) & 0XFF)
/*************************************************************************/
typedef unsigned char      u8;
typedef unsigned short int u16;
typedef short int          s16;
typedef int                s32;
/*************************************************************************/
#pragma pack(1)
/*************************************************************************/
//打开20ms数据上传结构体
union OPen20MsData
{
   u8 data[10];
   struct
   {
      u16 Header;
      u8  Len;
      u8  Type;
      u8  Cmd;
      u8  Num;
      u16 Data;
      u16 Check;
   }prot;
}Open20MsData;
/*************************************************************************/
union RxRobotData20MS
{
   u8 data[28];
   struct
   {
      u16 Header;       
      u8  Len;       
      u8  Type;         
      u8  Cmd;       
      u8  Num;       
      s16 FLSpeed;
      s16 FRSpeed;
      s16 BLSpeed;
      s16 BRSpeed;
      s16 FLAddEN; 
      s16 FRAddEN; 
      s16 BLAddEN; 
      s16 BRAddEN;
      u16 Voltage;
      u16 State;     
      u16 Check;        
   }prot;
}RXRobotData20MS;

/*************************************************************************/
//下发轮子速度结构体
union TXRobotData1
{
   u8 data[18];
   struct
   {
      u16 Header;
      u8  Len;
      u8  Type;
      u8  Cmd;
      u8  Num; 
      s16 FLSpeed;
      s16 FRSpeed;
      s16 BLSpeed;
      s16 BRSpeed;
      u16 StopCon;  
      u16 Check;
   }prot;
}TXRobotData1;
/*************************************************************************/
#pragma pack(4)

#endif
