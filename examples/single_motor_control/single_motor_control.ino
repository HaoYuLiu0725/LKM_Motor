#include <LKM_Motor.h>
/* LKM_Motor/examples/single_motor_control/single_motor_control.ino
This is the example of single motor control
Using Teensy4.0 and "Motor Communication Control" PCB in NTU ASR-LAB, and use "RS485_1" port on the PCB
("RS485_1" is using Serial5 for communication)

This example will control the motor turning from 0 to 60 degree, and then from 60 degree turn back to 0 degree.
*/

// create a LKM_Motor object
LKM_Motor motor1(1, 8, 5); // (id, reduction_ratio, serial_port)
/* Using "Motor Communication Control" PCB in NTU ASR-LAB, and use "RS485_1" port on the PCB, 
"RS485_1" is using Serial5 for communication */

void setup() {
  motor1.Serial_Init(); // initialize the motor's serial port
  Serial.begin(115200);
  pinMode(11, OUTPUT);  // LED2 on "Motor Communication Control" PCB in NTU ASR-LAB
  pinMode(12, OUTPUT);  // LED3 on "Motor Communication Control" PCB in NTU ASR-LAB
}

void loop() {
  digitalWrite(11, HIGH);                 // LED2 ON
  Serial.println("-> Go to 60 !");
  motor1.Write_Angle_MultiRound(60, 300); // send command to motor, go to 60 degree, speed = 300
  motor1.Print_Data();                    // print out motor's returned data (Temperature, Current, Velocity, Encoder position)
  delay(2000);
  digitalWrite(11, LOW);                  // LED2 OFF
  Serial.println("-> read ~");
  motor1.Read_Angle_MultiRound();         // send command to motor, read current angle
  motor1.Print_Angle();                   // print out motor's returned current angle data
  delay(2000);

  digitalWrite(12, HIGH);                 // LED3 ON
  Serial.println("-> Go to 0 !");
  motor1.Write_Angle_MultiRound(0, 300);  // send command to motor, go to 0 degree, speed = 300
  motor1.Print_Data();                    // print out motor's returned data (Temperature, Current, Velocity, Encoder position)
  delay(2000);
  digitalWrite(12, LOW);                  // LED3 OFF
  Serial.println("-> read ~");
  motor1.Read_Angle_MultiRound();         // send command to motor, read current angle
  motor1.Print_Angle();                   // print out motor's returned current angle data
  delay(2000);
}