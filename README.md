# LKM_Motor
LK tech's motor control library using Teensy4.0 with Arduino IDE.  
**Note:  
Recommand using "Motor Communication Control" PCB in NTU ASR-LAB,  
the "RS485_1" ports is using "Serial5" for communication,  
the "RS485_2" ports is using "Serial3" for communication,**

### Contents
* [Install](#install)
  * [Method1: Manually import the library](#method1-manually-import-the-library)
  * [Method2: Import the library through the Arduino IDE](#method2-import-the-library-through-the-arduino-ide)
* [Hardware Setup](#hardware-setup)
  * [Motor setup](#motor-setup)
  * [PCB setup](#pcb-setup)
* [Examples](#examples)
  * [Single motor control](#single-motor-control)
  * [Multi motor control](#multi-motor-control)
  * [Read motor angle](#read-motor-angle)
  * [Set motor origin](#set-motor-origin)
* [Library Methods](#library-methods)
* [Other](#other)
  * [Setup for Teensy 4.0 on Arduino IDE](#setup-for-teensy-40-on-arduino-ide)
  
## Install
### Method1: Manually import the library

1. Download library ZIP file, and extract it.
2. Place the extracted library folder in Documents(文件)/Arduino/libraries/.
3. Restart the Arduino IDE.
4. (Optional) Go to Documents(文件)/Arduino/libraries/LKM_Motor, you can delete "image" folder inside it.

### Method2: Import the library through the Arduino IDE

1. Download library ZIP file.
2. Open Arduino IDE, click on "Sketch" > "Include Library" > "Add .ZIP Library…", browse to find **"LKM_Motor-main.zip"** file, and click it.

<p align="center">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/import_library_with_zip.png" width="523" height="250">
</p>

3. The Arduino IDE will extract the archive, place the library in the Documents(文件)/Arduino/libraries/.
4. (Optional) Go to Documents(文件)/Arduino/libraries/, you can change folder name to "LKM_Motor", and also delete "image" folder inside it.

## Hardware Setup
### Motor setup

### PCB setup

## Examples
All examples utilize Teensy 4.0 and the 'Motor Communication Control' PCB in NTU ASR-LAB, employing the 'RS485_1' ports on the PCB.  
("RS485_1" is using Serial5 for communication)  
### Single motor control
[single_motor_control.ino](examples/single_motor_control/single_motor_control.ino)  
The example of single motor control  
This example will control the motor turning from 0 to 60 degree, and then from 60 degree turn back to 0 degree.  

### Multi motor control
[multi_motor_control.ino](examples/multi_motor_control/multi_motor_control.ino)  
The example of multiple motor control  
This example will control 3 motor turning from 0 to 60 degree, and then from 60 degree turn back to 0 degree.  

### Read motor angle
[read_motor_angle.ino](examples/read_motor_angle/read_motor_angle.ino)  
The example of read motor angle   
This example will read the angle of the motor specified by the user and print it, using 6 motors as an demonstration.  

### Set motor origin
[set_motor_origin.ino](examples/set_motor_origin/set_motor_origin.ino)  
The example of set motor origin and then read motor angle  
This example will set motor origin and then read the angle of the motor specified by the user and print it, using 6 motors as an demonstration.  

## Library Methods
`void Serial_Init()`

//更改馬達baudrate  
`void Change_Baudrate(int baudrate)`

//(8)設置馬達零點  
`void Set_Motor_Origin()`

//(9)讀取多圈角度命令  
`void Read_Angle_MultiRound()`

//(10)讀取單圈角度命令  
`void Read_Angle_SingleRound()`

//(15)電機關機命令  
`void Write_Motor_Shutdown()`

//(16)電機停止命令  
`void Write_Motor_Pause()`

//(17)電機運行命令  
`void Write_Motor_Run()`

//(19)轉矩閉環控制命令(current: -32~32 A)  
`void Write_Torque_Current(double current)`

//(20)速度閉環控制命令  
`void Write_Speed(double speed)`

//(21)多圈位置閉環控制命令1  
`void Write_Angle_MultiRound(double angle)`

//(22)多圈位置閉環控制命令2  
`void Write_Angle_MultiRound(double angle, double max_speed)`

//(23)單圈位置閉環控制命令1, direction: True -> 順時針 ; False -> 逆時針  
`void Write_Angle_SingleRound(bool direction, double angle)`

//(24)單圈位置閉環控制命令2, direction: True -> 順時針 ; False -> 逆時針  
`void Write_Angle_SingleRound(bool direction, double angle, double max_speed)`

//轉向自動-單圈位置閉環控制命令1(往角度小的方向走)  
`void Write_Angle_SingleRound(double angle)`

//轉向自動-單圈位置閉環控制命令2(往角度小的方向走)  
`void Write_Angle_SingleRound(double angle, double max_speed)`

//(25)增量位置閉環控制命令1  
`void Write_Angle_Increment(double angle_increment)`

//(26)增量位置閉環控制命令2  
`void Write_Angle_Increment(double angle_increment, double max_speed)`

//列印出馬達回傳的角度  
`void Print_Angle()`

//列印出馬達回傳的資料: 電機溫度、轉矩電流、電機速度以及編碼器位置  
`void Print_Data()`

`void Print_Setup_Data()`

`bool Find_Turn_Direction(double target_angle)`

## Other
### Setup for Teensy 4.0 on Arduino IDE

