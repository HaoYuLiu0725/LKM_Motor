#ifndef LKM_Motor_H
#define LKM_Motor_H

#include "Arduino.h"
#define BAUDRATE 4000000 //Use motor's MAX baudrate as default

class LKM_Motor
{
public:
  LKM_Motor();
  LKM_Motor(int id, int reduction_ratio, int serial_port);
  void Serial_Init();
  void Change_Baudrate(int baudrate);                                       //更改馬達baudrate
  void Set_Motor_Origin();                                                  //(8)設置馬達零點
  void Read_Angle_MultiRound();                                             //(9)讀取多圈角度命令
  void Read_Angle_SingleRound();                                            //(10)讀取單圈角度命令
  void Write_Motor_Shutdown();                                              //(15)電機關機命令
  void Write_Motor_Pause();                                                 //(16)電機停止命令
  void Write_Motor_Run();                                                   //(17)電機運行命令
  void Write_Torque_Current(int current);                                   //(19)轉矩閉環控制命令(current: -32~32 A)
  void Write_Speed(long speed);                                             //(20)速度閉環控制命令
  void Write_Angle_MultiRound(long angle);                                  //(21)多圈位置閉環控制命令1
  void Write_Angle_MultiRound(long angle, long max_speed);                  //(22)多圈位置閉環控制命令2
  void Write_Angle_SingleRound(bool direction, int angle);                  //(23)單圈位置閉環控制命令1, direction: True -> 順時針 ; False -> 逆時針
  void Write_Angle_SingleRound(bool direction, int angle, long max_speed);  //(24)單圈位置閉環控制命令2, direction: True -> 順時針 ; False -> 逆時針
  void Write_Angle_SingleRound(int angle);                                  //轉向自動-單圈位置閉環控制命令1(往角度小的方向走)
  void Write_Angle_SingleRound(int angle, long max_speed);                  //轉向自動-單圈位置閉環控制命令2(往角度小的方向走)
  void Write_Angle_Increment(int angle_increment);                          //(25)增量位置閉環控制命令1
  void Write_Angle_Increment(int angle_increment, int max_speed);           //(26)增量位置閉環控制命令2

  void Print_Angle(); //列印出馬達回傳的角度
  void Print_Data();  //列印出馬達回傳的資料: 電機溫度、轉矩電流、電機速度以及編碼器位置
  void Print_Setup_Data();
  bool Find_Turn_Direction(int target_angle);
  //儲存馬達回傳的資料
  int motor_id = 0;             //馬達ID
  double motor_angle = 0.0;     //馬達角度
  int8_t motor_temperature = 0; //馬達溫度
  int16_t motor_iq = 0;         //馬達轉矩電流
  int16_t motor_speed = 0;      //馬達轉速
  uint16_t motor_encoder = 0;   //馬達編碼器位置

private:
  void _Receive_Pack();                            //接受回傳指令
  void _Unpack(byte data_receive[30], int lenth);  //解讀封包內容
  Stream* MOTOR_SERIAL = &Serial1;
  int _id = 0;              //馬達設定的id
  int _reduction_ratio = 0; //馬達的減速比
  int _serial_port = 0;     //馬達使用的serial port
  int _baudrate = BAUDRATE; //馬達設定的baudrate
  byte _buffer[20];         //要傳送之封包
  int _count_RX = 0;        //計算儲存讀入封包之長度
  byte _readin[50];         //完整儲存讀入之封包
};

#endif