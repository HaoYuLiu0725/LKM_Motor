#include <LKM_Motor.h>


LKM_Motor MotorArr[14];

void setup() {
  motor1.Serial_Init(); 
  Serial.begin(115200);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  digitalWrite(11, HIGH);
  Serial.println("-> Go to 60 !");
  motor1.Write_Angle_MultiRound(60, 300);
  motor1.Print_Data();
  delay(2000);
  digitalWrite(11, LOW);
  Serial.println("-> read ~");
  motor1.Read_Angle_MultiRound();
  motor1.Print_Angle();
  delay(2000);
  digitalWrite(12, HIGH);
  Serial.println("-> Go to 0 !");
  motor1.Write_Angle_MultiRound(0, 300);
  motor1.Print_Data();
  delay(2000);
  digitalWrite(12, LOW);
  Serial.println("-> read ~");
  motor1.Read_Angle_MultiRound();
  motor1.Print_Angle();
  delay(2000);
}