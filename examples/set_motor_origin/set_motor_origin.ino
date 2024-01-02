#include <LKM_Motor.h>
/* LKM_Motor/examples/set_motor_origin/set_motor_origin.ino
This is the example of set motor origin and then read motor angle
Using Teensy4.0 and "Motor Communication Control" PCB in NTU ASR-LAB, and use "RS485_1" port on the PCB
("RS485_1" is using Serial5 for communication)

This example will set motor origin and then read the angle of the motor specified by the user and print it, using 6 motors as an demonstration.
*/

// define the number of motors
#define motor_num 6
// create a LKM_Motor object array
LKM_Motor MotorArr[motor_num];
// define want to "set motor origin" or not
bool set_motor_origin = true;

int motorID = 0; //specified by the user, use 1 ~ 6 in this example

void setup() {
  Serial.begin(115200);
  for(int i = 0; i < motor_num; i++){
    // assign LKM_Motor object into array. MotorArr[0] is motor ID 1, MotorArr[1] is motor ID 2, MotorArr[2] is motor ID 3, etc.
    MotorArr[i] = LKM_Motor(i+1, 8, 5); // (id, reduction_ratio, serial_port)
    /* Using "Motor Communication Control" PCB in NTU ASR-LAB, and use "RS485_1" port on the PCB, 
    "RS485_1" is using Serial5 for communication */
    MotorArr[i].Print_Setup_Data();     // print out motor's setup data (id, reduction_ratio, serial_port)
    MotorArr[i].Serial_Init();          // initialize the motor's serial port
  }
  if(set_motor_origin){
    for(int i = 0; i < motor_num; i++){
      MotorArr[i].Set_Motor_Origin();
    }
  }
}

void loop() {
  if(Serial.available()){                     // read user's input data (motorID) from Serial Monitor, use 1 ~ 6 in this example
    motorID = Serial.parseInt() - 1;
    Serial.println(motorID);
    Serial.clear();
  }
  MotorArr[motorID].Read_Angle_MultiRound();  // send command to motor, read current angle
  MotorArr[motorID].Print_Angle();            // print out motor's returned current angle data
  delay(100);
}
