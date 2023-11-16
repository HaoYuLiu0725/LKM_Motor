#include <LKM_Motor.h>
/* LKM_Motor/examples/multi_motor_control/multi_motor_control.ino
This is the example of multiple motor control
Using Teensy4.0 and "Motor Communication Control" PCB in NTU ASR-LAB, and use "RS485_1" ports on the PCB board
("RS485_1" is using Serial5 for communication)

This example will control 3 motor turning from 0 to 60 degree, and then from 60 degree turn back to 0 degree.
*/

// define the number of motors
#define motor_num 3 
// create a LKM_Motor object array
LKM_Motor MotorArr[motor_num];

char temp_buffer[100];

void setup() {
  Serial.begin(115200);
  for(int i = 0; i < motor_num; i++){
    // assign LKM_Motor object into array. MotorArr[0] is motor ID 1, MotorArr[1] is motor ID 2, MotorArr[2] is motor ID 3, etc.
    MotorArr[i] = LKM_Motor(i+1, 8, 5); // (id, reduction_ratio, serial_port)
    /* Using "Motor Communication Control" PCB board in NTU ASR-LAB, and use "RS485_1" port on the PCB board, 
    "RS485_1" is using Serial5 for communication */
    MotorArr[i].Print_Setup_Data();     // print out motor's setup data (id, reduction_ratio, serial_port)
    MotorArr[i].Serial_Init();          // initialize the motor's serial port
    MotorArr[i].Write_Motor_Run();      // (OPTIONAL) send command to motor to hold current position 
  }
}

void loop() {
  for(int i = 0; i < motor_num; i++){
    sprintf(temp_buffer, "-> Motor ID: %d go to 60 !", i+1);
    Serial.println(temp_buffer);
    MotorArr[i].Write_Angle_MultiRound(60, 300);              // send command to motor, go to 60 degree, speed = 300
    MotorArr[i].Print_Data();                                 // print out motor's returned data (Temperature, Current, Velocity, Encoder position)
    delay(2000);
    sprintf(temp_buffer, "-> Read Motor ID: %d's angle", i+1);
    Serial.println(temp_buffer);
    MotorArr[i].Read_Angle_MultiRound();                      // send command to motor, read current angle
    MotorArr[i].Print_Angle();                                // print out motor's returned current angle data
    delay(2000);

    sprintf(temp_buffer, "-> Motor ID: %d go to 0 !", i+1);
    Serial.println(temp_buffer);
    MotorArr[i].Write_Angle_MultiRound(0, 300);               // send command to motor, go to 0 degree, speed = 300
    MotorArr[i].Print_Data();                                 // print out motor's returned data (Temperature, Current, Velocity, Encoder position)
    delay(2000);
    Ssprintf(temp_buffer, "-> Read Motor ID: %d's angle", i+1);
    Serial.println(temp_buffer);
    MotorArr[i].Read_Angle_MultiRound();                      // send command to motor, read current angle
    MotorArr[i].Print_Angle();                                // print out motor's returned current angle data
    delay(2000);
  }
  delay(5000);
}
