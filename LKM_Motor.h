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
  void Change_Baudrate(int baudrate); //更改馬達baudrate
  void Write_Motor_Shutdown();                                                  //(5)電機關機命令(0x80)
  void Write_Motor_Run();                                                       //(6)電機運行命令(0x88)
  void Write_Motor_Pause();                                                     //(7)電機停止命令(0x81)
  void Write_Torque_Current(double current);                                    //(10)轉矩閉環控制命令(0xA1), current: -32~32 A
  void Write_Speed(double speed);                                               //(11)速度閉環控制命令(0xA2)
  void Write_Angle_MultiRound(double angle);                                    //(12)多圈位置閉環控制命令1(0xA3)
  void Write_Angle_MultiRound(double angle, double max_speed);                  //(13)多圈位置閉環控制命令2(0xA4)
  void Write_Angle_SingleRound(bool direction, double angle);                   //(14)單圈位置閉環控制命令1(0xA5), direction: True -> 順時針 ; False -> 逆時針
  void Write_Angle_SingleRound(bool direction, double angle, double max_speed); //(15)單圈位置閉環控制命令2(0xA6), direction: True -> 順時針 ; False -> 逆時針
  void Write_Angle_SingleRound(double angle);                                   //轉向自動-單圈位置閉環控制命令1(往角度小的方向走)
  void Write_Angle_SingleRound(double angle, double max_speed);                 //轉向自動-單圈位置閉環控制命令2(往角度小的方向走)
  void Write_Angle_Increment(double angle_increment);                           //(16)增量位置閉環控制命令1(0xA7)
  void Write_Angle_Increment(double angle_increment, double max_speed);         //(17)增量位置閉環控制命令2(0xA8)
  void Set_Motor_Origin();                                                      //(19)設置馬達零點(0x19)
  void Read_Angle_MultiRound();                                                 //(20)讀取多圈角度命令(0x92)
  void Read_Angle_SingleRound();                                                //(22)讀取單圈角度命令(0x94)

  void Print_Angle();             //列印出馬達回傳的角度
  void Print_Data();              //列印出馬達回傳的資料: 電機溫度、轉矩電流、電機速度以及編碼器位置
  void Print_Setup_Data();
  void Print_Angle_Custom();      //列印出馬達回傳的角度, 角度的負值經過計算處理
  void Print_PID_Param();         //列印出馬達回傳的PID參數
  bool Find_Turn_Direction(double target_angle);
  void Calculate_Custom_Angle();  // 角度的負值經過計算處理
  
  /*儲存馬達回傳的資料*/
  int motor_id = 0;                 //馬達ID
  double motor_angle = 0.0;         //馬達角度
  int8_t motor_temperature = 0;     //馬達溫度
  int16_t motor_iq = 0;             //馬達轉矩電流
  int16_t motor_speed = 0;          //馬達轉速
  uint16_t motor_encoder = 0;       //馬達編碼器位置
  double motor_angle_custom = 0.0;  //馬達角度, 角度的負值經過計算處理
  int motor_angle_Kp = 0;           //馬達PID參數-角度Kp
  int motor_angle_Ki = 0;           //馬達PID參數-角度Ki
  int motor_speed_Kp = 0;           //馬達PID參數-速度Kp
  int motor_speed_Ki = 0;           //馬達PID參數-速度Ki
  int motor_iq_Kp = 0;              //馬達PID參數-轉矩Kp
  int motor_iq_Ki = 0;              //馬達PID參數-轉矩Ki

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