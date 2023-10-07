#include <LKM_Motor.h>

// create a LKM_Motor object
LKM_Motor motor1(1, 8, 5); // (id, reduction_ratio, serial_port)

void setup() {
  motor1.Serial_Init(); // initialize the motor's serial port
  Serial.begin(115200);
  pinMode(11, OUTPUT); //LED2 on "Motor Communication Control" PCB board in NTU ASR-LAB
  pinMode(12, OUTPUT); //LED3 on "Motor Communication Control" PCB board in NTU ASR-LAB
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