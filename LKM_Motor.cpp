#include "Arduino.h"
#include "LKM_Motor.h"
LKM_Motor::LKM_Motor(){
  _id = 0;
  _reduction_ratio = 0;
  _serial_port = 0;
}

LKM_Motor::LKM_Motor(int id, int reduction_ratio, int serial_port){
  _id = id;
  _reduction_ratio = reduction_ratio;
  _serial_port = serial_port;
}

void LKM_Motor::Serial_Init(){
  if(_serial_port == 1){
    MOTOR_SERIAL = &Serial1;
    Serial1.begin(_baudrate);
  }
  else if(_serial_port == 2){
    MOTOR_SERIAL = &Serial2;
    Serial2.begin(_baudrate);
  }
  else if(_serial_port == 3){
    MOTOR_SERIAL = &Serial3;
    Serial3.begin(_baudrate);
    /* If using "Motor Communication Control" PCB board in NTU ASR-LAB,
    the RS485_2 is using Serial3 for communication,
    and use GPIO-13 to switch between TX and RX. */
    Serial3.transmitterEnable(13); 
  }
  else if(_serial_port == 4){
    MOTOR_SERIAL = &Serial4;
    Serial4.begin(_baudrate);
  }
  else if(_serial_port == 5){
    MOTOR_SERIAL = &Serial5;
    Serial5.begin(_baudrate);
    /* If using "Motor Communication Control" PCB board in NTU ASR-LAB,
    the RS485_1 is using Serial5 for communication,
    and use GPIO-2 to switch between TX and RX. */
    Serial5.transmitterEnable(2);
  }
  else if(_serial_port == 6){
    MOTOR_SERIAL = &Serial6;
    Serial6.begin(_baudrate);
  }
  else if(_serial_port == 7){
    MOTOR_SERIAL = &Serial7;
    Serial7.begin(_baudrate);
  }
  else{
    Serial.println("===== Serial Port Error! =====");
  }
}

void LKM_Motor::Change_Baudrate(int baudrate){
  _baudrate = baudrate;
  Serial_Init();
}

//(1)讀取PID參數
void LKM_Motor::Read_PID_Param(){
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x30;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x00;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;            //幀頭校驗字節
  MOTOR_SERIAL->write(_buffer, 5);  //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(2)寫入PID到RAM, 斷電後寫入參數失效
void LKM_Motor::Write_PID_Into_RAM(int anglePidKp, int anglePidKi, int speedPidKp, int speedPidKi, int iqPidKp, int iqPidKi){
  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x31;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x06;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;    //幀頭校驗字節
  _buffer[5] = anglePidKp;  //位置環P參數
  _buffer[6] = anglePidKi;  //位置環I參數
  _buffer[7] = speedPidKp;  //速度環P參數
  _buffer[8] = speedPidKi;  //速度環I參數
  _buffer[9] = iqPidKp;     //轉矩環P參數
  _buffer[10] = iqPidKi;    //轉矩環I參數
  checkSum = 0;
  for (int i = 5; i <= 10; i++){
    checkSum += _buffer[i];
  }
  _buffer[11] = checkSum;           //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 12); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(3)讀取PID到ROM, 斷電後寫入參數仍然有效
void LKM_Motor::Write_PID_Into_ROM(int anglePidKp, int anglePidKi, int speedPidKp, int speedPidKi, int iqPidKp, int iqPidKi){
  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x32;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x06;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;    //幀頭校驗字節
  _buffer[5] = anglePidKp;  //位置環P參數
  _buffer[6] = anglePidKi;  //位置環I參數
  _buffer[7] = speedPidKp;  //速度環P參數
  _buffer[8] = speedPidKi;  //速度環I參數
  _buffer[9] = iqPidKp;     //轉矩環P參數
  _buffer[10] = iqPidKi;    //轉矩環I參數
  checkSum = 0;
  for (int i = 5; i <= 10; i++){
    checkSum += _buffer[i];
  }
  _buffer[11] = checkSum;           //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 12); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(8)設置馬達零點
void LKM_Motor::Set_Motor_Origin(){
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x19;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x00;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;            //幀頭校驗字節
  MOTOR_SERIAL->write(_buffer, 5);  //送出封包
}

//(9)讀取多圈角度命令
void LKM_Motor::Read_Angle_MultiRound(){
  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x92;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x00;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;            //幀頭校驗字節
  MOTOR_SERIAL->write(_buffer, 5);  //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(10)讀取單圈角度命令
void LKM_Motor::Read_Angle_SingleRound(){
  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x94;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x00;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;            //幀頭校驗字節
  MOTOR_SERIAL->write(_buffer, 5);  //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(15)電機關機命令
void LKM_Motor::Write_Motor_Shutdown(){
  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x80;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x00;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;            //幀頭校驗字節
  MOTOR_SERIAL->write(_buffer, 5);  //送出封包
}

//(16)電機停止命令
void LKM_Motor::Write_Motor_Pause(){
  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x81;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x00;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;            //幀頭校驗字節
  MOTOR_SERIAL->write(_buffer, 5);  //送出封包
}

//(17)電機運行命令
void LKM_Motor::Write_Motor_Run(){
  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0x88;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x00;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;            //幀頭校驗字節
  MOTOR_SERIAL->write(_buffer, 5);  //送出封包
}

//(19)轉矩閉環控制命令(current: -32~32 A)
void LKM_Motor::Write_Torque_Current(double current){
  int16_t iqControl = (int16_t)(current * 2000 / 32);

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA1;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x02;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = (iqControl>>0) & 0xFF; //轉矩電流控制值低字節
  _buffer[6] = (iqControl>>8) & 0xFF; //轉矩電流控制值高字節
  checkSum = 0;
  for (int i = 5; i <= 6; i++){
    checkSum += _buffer[i];
  }
  _buffer[7] = checkSum;            //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 8);  //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(20)速度閉環控制命令
void LKM_Motor::Write_Speed(double speed){
  int32_t speedControl = (int32_t)(speed * 100 * _reduction_ratio);

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA2;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x04;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = (speedControl>>0) & 0xFF;  //電機速度低字節
  _buffer[6] = (speedControl>>8) & 0xFF;
  _buffer[7] = (speedControl>>16) & 0xFF;
  _buffer[8] = (speedControl>>24) & 0xFF; //電機速度高字節
  checkSum = 0;
  for (int i = 5; i <= 8; i++){
    checkSum += _buffer[i];
  }
  _buffer[9] = checkSum;            //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 10); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(21)多圈位置閉環控制命令1
void LKM_Motor::Write_Angle_MultiRound(double angle){
  int64_t angleControl = (int64_t)(angle * 100 * _reduction_ratio);   //單位換算與乘上齒輪比

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA3;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x08;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = (angleControl>>0) & 0xFF;  //5~12 位置
  _buffer[6] = (angleControl>>8) & 0xFF;
  _buffer[7] = (angleControl>>16) & 0xFF;
  _buffer[8] = (angleControl>>24) & 0xFF;
  _buffer[9] = (angleControl>>32) & 0xFF;
  _buffer[10] = (angleControl>>40) & 0xFF;
  _buffer[11] = (angleControl>>48) & 0xFF;
  _buffer[12] = (angleControl>>56) & 0xFF;
  checkSum = 0;
  for (int i = 5; i <= 12; i++){
    checkSum += _buffer[i];
  }
  _buffer[13] = checkSum;           //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 14); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(22)多圈位置閉環控制命令2
void LKM_Motor::Write_Angle_MultiRound(double angle, double max_speed){
  int64_t angleControl = (int64_t)(angle * 100 * _reduction_ratio);   //單位換算與乘上齒輪比
  uint32_t maxSpeed = (uint32_t)(max_speed * 100 * _reduction_ratio); //單位換算與乘上齒輪比

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA4;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x0C;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = (angleControl>>0) & 0xFF;  //5~12 位置
  _buffer[6] = (angleControl>>8) & 0xFF;
  _buffer[7] = (angleControl>>16) & 0xFF;
  _buffer[8] = (angleControl>>24) & 0xFF;
  _buffer[9] = (angleControl>>32) & 0xFF;
  _buffer[10] = (angleControl>>40) & 0xFF;
  _buffer[11] = (angleControl>>48) & 0xFF;
  _buffer[12] = (angleControl>>56) & 0xFF;
  _buffer[13] = (maxSpeed>>0) & 0xFF;     //13~16 速度
  _buffer[14] = (maxSpeed>>8) & 0xFF;
  _buffer[15] = (maxSpeed>>16) & 0xFF;
  _buffer[16] = (maxSpeed>>24) & 0xFF;
  checkSum = 0;
  for (int i = 5; i <= 16; i++){
    checkSum += _buffer[i];
  }
  _buffer[17] = checkSum;           //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 18); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(23)單圈位置閉環控制命令1, direction: True -> 順時針 ; False -> 逆時針
void LKM_Motor::Write_Angle_SingleRound(bool direction, double angle){
  uint8_t spinDirection = direction ? 0x00 : 0x01; //True -> 0x00順時針 ; False -> 0x01逆時針
  uint16_t angleControl = (uint16_t)(angle * 100 * _reduction_ratio);   //單位換算與乘上齒輪比

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA5;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x04;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = spinDirection;             //轉動方向
  _buffer[6] = (angleControl>>0) & 0xFF;  //位置控制低字節
  _buffer[7] = (angleControl>>8) & 0xFF;  //位置控制高字節
  _buffer[8] = 0x00;                      //NULL;
  checkSum = 0;
  for (int i = 5; i <= 8; i++){
    checkSum += _buffer[i];
  }
  _buffer[9] = checkSum;            //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 10); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(24)單圈位置閉環控制命令2, direction: True -> 順時針 ; False -> 逆時針
void LKM_Motor::Write_Angle_SingleRound(bool direction, double angle, double max_speed){
  uint8_t spinDirection = direction ? 0x00 : 0x01; //True -> 0x00順時針 ; False -> 0x01逆時針
  uint16_t angleControl = (uint16_t)(angle * 100 ); //單位換算與乘上齒輪比 * _reduction_ratio
  uint32_t maxSpeed = (uint32_t)(max_speed * 100 * _reduction_ratio); //單位換算與乘上齒輪比

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA6;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x08;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = spinDirection;             //轉動方向
  _buffer[6] = (angleControl>>0) & 0xFF;  //位置控制低字節
  _buffer[7] = (angleControl>>8) & 0xFF;  //位置控制高字節
  _buffer[8] = 0x00;                      //NULL
  _buffer[9] = (maxSpeed>>0) & 0xFF;      //9~12 速度
  _buffer[10] = (maxSpeed>>8) & 0xFF;
  _buffer[11] = (maxSpeed>>16) & 0xFF;
  _buffer[12] = (maxSpeed>>24) & 0xFF;
  checkSum = 0;
  for (int i = 5; i <= 12; i++){
    checkSum += _buffer[i];
  }
  _buffer[13] = checkSum;           //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 14); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//轉向自動-單圈位置閉環控制命令1(往角度小的方向走)
void LKM_Motor::Write_Angle_SingleRound(double angle){
  bool direction = true;
  direction = Find_Turn_Direction(angle);
  Write_Angle_SingleRound(direction, angle);
}

//轉向自動-單圈位置閉環控制命令2(往角度小的方向走)
void LKM_Motor::Write_Angle_SingleRound(double angle, double max_speed){
  bool direction = true;
  direction = Find_Turn_Direction(angle);
  Write_Angle_SingleRound(direction, angle, max_speed);
}

//(25)增量位置閉環控制命令1
void LKM_Motor::Write_Angle_Increment(double angle_increment){
  int32_t angleIncrement = (int32_t)(angle_increment * 100 * _reduction_ratio);   //單位換算與乘上齒輪比

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA7;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x04;  //數據長度字節 (datasheet is 0x03 ?????)
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = (angleIncrement>>0) & 0xFF;  //5~8 增量位置
  _buffer[6] = (angleIncrement>>8) & 0xFF;
  _buffer[7] = (angleIncrement>>16) & 0xFF;
  _buffer[8] = (angleIncrement>>24) & 0xFF;
  checkSum = 0;
  for (int i = 5; i <= 8; i++){
    checkSum += _buffer[i];
  }
  _buffer[9] = checkSum;            //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 10); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//(26)增量位置閉環控制命令2
void LKM_Motor::Write_Angle_Increment(double angle_increment, double max_speed){
  int32_t angleIncrement = (int32_t)(angle_increment * 100 * _reduction_ratio); //單位換算與乘上齒輪比
  uint32_t maxSpeed = (uint32_t)(max_speed * 100 * _reduction_ratio);           //單位換算與乘上齒輪比

  // 發送封包
  byte checkSum;
  _buffer[0] = 0x3E;  //頭字節
  _buffer[1] = 0xA8;  //命令字節
  _buffer[2] = _id;   //ID
  _buffer[3] = 0x08;  //數據長度字節
  checkSum = 0;
  for (int i = 0; i <= 3; i++){
    checkSum += _buffer[i];
  }
  _buffer[4] = checkSum;  //幀頭校驗字節
  _buffer[5] = (angleIncrement>>0) & 0xFF;  //5~8 增量位置
  _buffer[6] = (angleIncrement>>8) & 0xFF;
  _buffer[7] = (angleIncrement>>16) & 0xFF;
  _buffer[8] = (angleIncrement>>24) & 0xFF;
  _buffer[9] = (maxSpeed>>0) & 0xFF;        //9~12 速度
  _buffer[10] = (maxSpeed>>8) & 0xFF;
  _buffer[11] = (maxSpeed>>16) & 0xFF;
  _buffer[12] = (maxSpeed>>24) & 0xFF;
  checkSum = 0;
  for (int i = 5; i <= 12; i++){
    checkSum += _buffer[i];
  }
  _buffer[13] = checkSum;           //數據校驗字節
  MOTOR_SERIAL->write(_buffer, 14); //送出封包
  _Receive_Pack();                  //接收電機回覆
}

//接受回傳指令
void LKM_Motor::_Receive_Pack(){
  byte temp[1]; //每次接收進來之byte
  byte checkSum;

  _count_RX = 0;
  while(MOTOR_SERIAL->available() > 0){
    MOTOR_SERIAL->readBytes(temp, 1); //讀取輸入之byte
    // Serial.println(temp[0]);
    // 確認是否為頭字節
    if(temp[0] == 0x3E){
      // Serial.println("head");
      _readin[0] = temp[0];
      _count_RX = 1;
    }
    // 幀頭校驗字節
    else if(_count_RX == 4){
      // Serial.println("headercheck");
      _readin[_count_RX] = temp[0];
      
      checkSum = 0;
      for(int i = 0; i <= 3; i++){
        checkSum += _readin[i];
      }
      if(checkSum != _readin[_count_RX]){
        Serial.println("===== Packet Error: Header Checksum Error! =====");
        break;
      }
      _count_RX++;
    }
    // 數據校驗字節
    else if(_count_RX > 4 && _count_RX == _readin[3]+5){
      // Serial.println("datacheck");
      _readin[_count_RX] = temp[0];
      
      checkSum = 0;
      for(int i = 5; i <= _count_RX-1; i++){
        checkSum += _readin[i];
      }
      
      if(checkSum != _readin[_count_RX]){
        Serial.println("===== Packet Error: Data Checksum Error! =====");
        break;
      }
      else{
        // Serial.println("Unpack");
        _Unpack(_readin, _count_RX); //傳送去解讀封包
        break;
      }
    }
    else{
      // Serial.println("data");
      _readin[_count_RX] = temp[0];
      _count_RX++;
    }
  }
  while(MOTOR_SERIAL->read() >= 0){} //clear serialbuffer
}

// 解讀封包內容
void LKM_Motor::_Unpack(byte data_receive[30], int lenth){
  // (1)讀取PID參數 (0x30)
  // (2)寫入PID到RAM, 斷電後寫入參數失效 (0x31)
  // (3)讀取PID到ROM, 斷電後寫入參數仍然有效 (0x32)
  if (data_receive[1] == 0x30 || data_receive[1] == 0x31 || data_receive[1] == 0x32){
    motor_id = (int)data_receive[2]; //馬達ID
    motor_angle_Kp = (int)data_receive[5];  //馬達PID參數-角度Kp
    motor_angle_Ki = (int)data_receive[6];  //馬達PID參數-角度Ki
    motor_speed_Kp = (int)data_receive[7];  //馬達PID參數-速度Kp
    motor_speed_Ki = (int)data_receive[8];  //馬達PID參數-速度Ki
    motor_iq_Kp = (int)data_receive[9];     //馬達PID參數-轉矩Kp
    motor_iq_Ki = (int)data_receive[10];    //馬達PID參數-轉矩Ki
  }
  // (9)讀取多圈角度命令
  else if (data_receive[1] == 0x92){
    motor_id = (int)data_receive[2]; //馬達ID
    int64_t motorAngle = (int64_t)(((int64_t)data_receive[12]<<56) + ((int64_t)data_receive[11]<<48) + ((int64_t)data_receive[10]<<40) + 
    ((int64_t)data_receive[9]<<32) + (data_receive[8]<<24) + (data_receive[7]<<16) + (data_receive[6]<<8) + data_receive[5]);
    motor_angle = (double)motorAngle / (100.0 * (double)_reduction_ratio);
    Calculate_Custom_Angle();
  }
  // (10)讀取單圈角度命令
  else if (data_receive[1] == 0x94){
    motor_id = (int)data_receive[2]; //馬達ID
    uint32_t circleAngle = (uint32_t)((data_receive[8]<<24) + (data_receive[7]<<16) + (data_receive[6]<<8) + data_receive[5]);
    motor_angle = (double)circleAngle / (100.0 * (double)_reduction_ratio);
  }
  // (19)轉矩閉環控制命令: 回傳電機溫度、電機轉矩電流值、電機轉速以及編碼器位置 (0xA1)
  // (20)速度閉環控制命令: 回傳電機溫度、轉矩電流、電機速度以及編碼器位置 (0xA2)
  // (21)多圈位置閉環控制命令1: 回傳電機溫度、轉矩電流、電機速度以及編碼器位置 (0xA3)
  // (22)多圈位置閉環控制命令2: 回傳電機溫度、轉矩電流、電機速度以及編碼器位置 (0xA4)
  // (23)單圈位置閉環控制命令1: 回傳電機溫度、轉矩電流、電機速度以及編碼器位置 (0xA5)
  // (24)單圈位置閉環控制命令2: 回傳電機溫度、轉矩電流、電機速度以及編碼器位置 (0xA6)
  // (25)增量位置閉環控制命令1: 回傳電機溫度、轉矩電流、電機速度以及編碼器位置 (0xA7)
  // (26)增量位置閉環控制命令2: 回傳電機溫度、轉矩電流、電機速度以及編碼器位置 (0xA8)
  else if (data_receive[1] == 0xA1 || data_receive[1] == 0xA2 || data_receive[1] == 0xA3 || data_receive[1] == 0xA4 || data_receive[1] == 0xA5 || data_receive[1] == 0xA6 || data_receive[1] == 0xA7 || data_receive[1] == 0xA8){
    motor_id = (int)data_receive[2]; //馬達ID
    motor_temperature = (int8_t)(data_receive[5]);
    int16_t iq_current = (int16_t)((data_receive[7]<<8) + (data_receive[6]));
    motor_iq = (double)iq_current * 33.0 / 2048.0;
    motor_speed = (int16_t)((data_receive[9]<<8) + (data_receive[8]));
    motor_encoder = (uint16_t)((data_receive[11]<<8) + (data_receive[10]));
  }
}

void LKM_Motor::Print_Angle(){
  Serial.print("Motor ID: "); Serial.println(motor_id);
  Serial.print("Angle:    "); Serial.println(motor_angle);
}

void LKM_Motor::Print_Data(){
  Serial.print("Motor ID:         "); Serial.println(motor_id);
  Serial.print("Temperature:      "); Serial.println(motor_temperature);
  Serial.print("Current:          "); Serial.println(motor_iq);
  Serial.print("Velocity:         "); Serial.println(motor_speed); 
  Serial.print("Encoder position: "); Serial.println(motor_encoder);
}

void LKM_Motor::Print_Setup_Data(){
  char temp_buffer[100];
  sprintf(temp_buffer, "id: %d, reduction_ratio: %d, serial_port: %d", _id, _reduction_ratio, _serial_port);
  Serial.println(temp_buffer);
}

void LKM_Motor::Print_Angle_Custom(){
  Serial.print("Motor ID:     "); Serial.println(motor_id);
  Serial.print("Custom Angle: "); Serial.println(motor_angle_custom);
}

void LKM_Motor::Print_PID_Param(){
  char temp_buffer[100];
  sprintf(temp_buffer, "angle_Kp: %d, angle_Ki: %d, speed_Kp: %d, speed_Ki: %d, iq_Kp: %d, iq_Ki: %d", motor_angle_Kp, motor_angle_Ki, motor_speed_Kp, motor_speed_Ki, motor_iq_Kp, motor_iq_Ki);
  Serial.print("Motor ID: "); Serial.println(motor_id);
  Serial.println(temp_buffer);
}

// direction: True -> 順時針 ; False -> 逆時針
bool LKM_Motor::Find_Turn_Direction(double target_angle){
  Read_Angle_SingleRound();
  double diff = target_angle - (double)motor_angle;
  if(diff < 0) diff += 360.0;
  if(diff > 180) return false; // counterclockwise 逆時針
  else           return true;  // clockwise 順時針
}

void LKM_Motor::Calculate_Custom_Angle(){
  if(motor_angle < 0) motor_angle_custom = motor_angle + (pow(2, 32) / 800);
  else motor_angle_custom = motor_angle;
}