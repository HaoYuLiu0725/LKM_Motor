#ifndef LKM_Motor_H
#define LKM_Motor_H

#include "Arduino.h"
#define LKM_Motor_BAUDRATE 4000000 //Use motor's MAX baudrate as default

/* Communication Protocol Manual Version: V2.35 */
/* Note:
    If using "Motor Communication Control" PCB board in NTU ASR-LAB,
    the RS485_1 is using Serial5 for communication, and use GPIO-2 to switch between TX and RX.
    the RS485_2 is using Serial3 for communication, and use GPIO-13 to switch between TX and RX.
*/

class LKM_Motor
{
public:
  LKM_Motor();
  LKM_Motor(int id, int reduction_ratio, int serial_port);
  LKM_Motor(int id, int reduction_ratio, int serial_port, double Kt);
  void Serial_Init();
  void Change_Baudrate(int baudrate);                                           //更改馬達baudrate
  void Read_Motor_State_2();                                                    //(3)讀取電機狀態2命令(0x9C)
  void Write_Motor_Shutdown();                                                  //(5)電機關機命令(0x80)
  void Write_Motor_Run();                                                       //(6)電機運行命令(0x88)
  void Write_Motor_Pause();                                                     //(7)電機停止命令(0x81)
  void Write_Torque_Current(double current);                                    //(10)轉矩閉環控制命令(0xA1), current: -32~32
  void Write_Torque(double torque);                                             //轉矩控制, torque [N*m] = Kt * I
  void Write_Speed(double speed);                                               //(11)速度閉環控制命令(0xA2)
  void Write_Angle_MultiRound(double angle);                                    //(12)多圈位置閉環控制命令1(0xA3)
  void Write_Angle_MultiRound(double angle, double max_speed);                  //(13)多圈位置閉環控制命令2(0xA4)
  void Write_Angle_SingleRound(double angle, bool direction);                   //(14)單圈位置閉環控制命令1(0xA5), direction: True -> 順時針 ; False -> 逆時針
  void Write_Angle_SingleRound(double angle, double max_speed, bool direction); //(15)單圈位置閉環控制命令2(0xA6), direction: True -> 順時針 ; False -> 逆時針
  void Write_Angle_SingleRound(double angle);                                   //轉向自動-單圈位置閉環控制命令1(往角度小的方向走)
  void Write_Angle_SingleRound(double angle, double max_speed);                 //轉向自動-單圈位置閉環控制命令2(往角度小的方向走)
  void Write_Angle_Increment(double angle_increment);                           //(16)增量位置閉環控制命令1(0xA7)
  void Write_Angle_Increment(double angle_increment, double max_speed);         //(17)增量位置閉環控制命令2(0xA8)
  void Set_Motor_Origin();                                                      //(19)設置馬達零點(0x19)
  void Read_Angle_MultiRound();                                                 //(20)讀取多圈角度命令(0x92)
  void Read_Angle_SingleRound();                                                //(22)讀取單圈角度命令(0x94)
  // void Read_Setup_Param(uint8_t ParamID);                                          //(24)讀取設定參數命令(0x40)
  // void Read_PID_Param();                                                        //讀取PID參數
  // //(25)寫入設定參數到RAM(0x42), 斷電後失效
  // void Write_Setup_Param_Into_RAM(uint8_t ParamID, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6);
  // //(26)寫入設定參數到ROM(0x44), 斷電後仍然有效
  // void Write_Setup_Param(uint8_t ParamID, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6);

  void Print_Setup_Data();        //列印出馬達設定的 id, reduction_ratio, serial_port
  void Print_Data();              //列印出馬達回傳的資料: 電機溫度、轉矩電流、電機速度以及編碼器位置
  void Print_Angle();             //列印出馬達回傳的角度
  void Print_PID_Param();         //列印出馬達的PID參數
  void Set_Need_Receive(bool need_receive); //設定非讀取資訊的指令是否需要解封包
  void Set_Kt(double Kt);         //設定馬達的轉矩常數Kt

  int delay_time = 250; //[us]
  bool debug_mode = true; //decide whether to use Serial print debug message in _Receive_Pack() function
  
  /*儲存馬達回傳的資料*/
  int motor_id = 0;                 //馬達ID
  int8_t motor_temperature = 0;     //馬達溫度
  double motor_iq = 0;              //馬達轉矩電流
  int16_t motor_speed = 0;          //馬達轉速
  uint16_t motor_encoder = 0;       //馬達編碼器位置
  double motor_angle_encoder = 0.0; //編碼器角度 (-180 ~ 180)
  double motor_angle_single = 0.0;  //單圈馬達角度 (-180 ~ 180)
  double motor_angle_multi = 0.0;   //多圈馬達角度

  uint16_t anglePidKp = 0;    //角度環Kp
  uint16_t anglePidKi = 0;    //角度環Ki
  uint16_t anglePidKd = 0;    //角度環Kd
  uint16_t speedPidKp = 0;    //速度環Kp
  uint16_t speedPidKi = 0;    //速度環Ki
  uint16_t speedPidKd = 0;    //速度環Kd
  uint16_t currentPidKp = 0;  //電流環Kp
  uint16_t currentPidKi = 0;  //電流環Ki
  uint16_t currentPidKd = 0;  //電流環Kd
  
private:
  void _Receive_Pack(const int pack_length); //接受回傳指令
  void _Unpack(uint8_t data_receive[30]);    //解讀封包內容
  bool _Find_Turn_Direction(double target_angle);  // direction: True -> 順時針 ; False -> 逆時針
  double _Normalize_Angle_Deg(double angle);       // 將任意角度轉化成 -180 ~ 180
  // void _Unpack_Read_Setup_Param(uint8_t data_receive[30]); //解讀回傳的設定參數
  // void _Unpack_Write_Setup_Param(uint8_t data_receive[30]);//解讀設定參數是否成功
  Stream* MOTOR_SERIAL = &Serial1;
  int _id = 0;                        //馬達設定的id
  int _reduction_ratio = 0;           //馬達的減速比
  int _serial_port = 0;               //馬達使用的serial port
  double _Kt = 0.0;                   //馬達的轉矩常數Kt
  int _baudrate = LKM_Motor_BAUDRATE; //馬達設定的baudrate
  bool _need_receive = false;         //設定非讀取資訊的指令是否需要解封包
};

#endif