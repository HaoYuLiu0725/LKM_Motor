#include <LKM_Motor.h>

#define motor_num 4 
// create a LKM_Motor object array
LKM_Motor MotorArr[motor_num];

void setup() {
  Serial.begin(115200);
  for(int i = 0; i < motor_num; i++){
    MotorArr[i] = LKM_Motor(i+1, 8, 5); // (id, reduction_ratio, serial_port)
    MotorArr[i].Print_Setup_Data();
    MotorArr[i].Serial_Init();
    MotorArr[i].Write_Motor_Run();
  }
}

void loop() {
  
}