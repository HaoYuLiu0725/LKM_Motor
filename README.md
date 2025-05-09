# LKM_Motor
LK tech's motor control library using Teensy4.0 with Arduino IDE.  
**Note:  
Recommand using "Motor Communication Control" PCB in NTU ASR-LAB,  
the "RS485_1" ports are using "Serial5" for communication,  
the "RS485_2" ports are using "Serial3" for communication,**

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

1. Prepare LK motor, power cable, RS485 cable, and power supply / 24V or 48V battery.
2. Connect the wires as shown in the image below.

<p align="center">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/single_motor.png" width="400" height="300">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/double_motor.png" width="400" height="300">
</p>

3. Prepare one of the following two sets of items.:  
  A. **U2D2 module with a modified signal line that can be directly connected to RS485** + USB Type-A to Micro-B cable.  
  B. **USB to UART module that provide by LK-tech** + USB Type-A to Type-C cable.

<p align="center">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/U2D2_module.png" width="400" height="300">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/U2D2_module_modified.png" width="400" height="300">
 
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/LK_UART_module.png" width="400" height="300">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/LK_UART_module_close.png" width="400" height="300">
</p>

4. Use either set A or set B to connect the motor and your PC.
5. Open the setting tool (LK moto.V2.63.exe) in the "setting_tool" folder, and power up the motor.
6. Choose a COM port that you connect the motor and your PC.

<p align="center">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/setting_tool_COM.png" width="400" height="300">
</p>

7. Using a default Baudrate of 115200 and ID 1, press "CONNECT" button connect to the motor. If the motor is new, you should successfully connect to the motor.
8. If the motor's communication connection is **successful**, you will see the screen on the **left picture**, and you can proceed to **step 12** to continue the setup.  
   BUT if the motor's communication connection **fails**, you will see the error screen on the **right picture**. Please continue with **step 9** to attempting the connection.

<p align="center">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/setting_tool_default.png" width="400" height="300">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/setting_tool_error.png" width="400" height="300">
</p>

9. (The motor's communication connection has **failed**, indicating that the motor's settings has been modified)  
   If you are using **set A** (U2D2 module with a modified signal line that can be directly connected to RS485 + USB Type-A to Micro-B cable), proceed to **step 10** to continue attempting the connection.  
   If you are using **set B** (USB to UART module that provide by LK-tech + USB Type-A to Type-C cable), proceed to **step 11** to continue attempting the connection. 
10. (**Using set A**)  
    If you know the baud rate and ID that this motor is using, try changing the baud rate and ID at the top of the setting tool screen and connect again.  
    If the connection still fails, switch to **set B** and use the default baud rate of 115200 and ID 1 to connect to the motor.  
    If it still doesn't work, proceed to **step 11** to attempt the connection.
11. (**Using set B**)
    The UART baud rate can't be changed, so still use the default baud rate of 115200 and try different IDs (1~20) to connect to the motor.
    If it still doesn't work, it means the motor is broken or something you are doing is wrong. Please reach out to the seniors in the ASR Lab for help.
12. (The motor's communication connection is **successful**, proceed to continue the setup)  
    Change the settings as shown in the image below.  
    i. Change the "Driver ID" to the motor number you want to set.  
    ii. Change the "RS485 Baudrate" to the baud rate you want to set. It is recommended to use 4000000 (4M) as it is the fastest.  
    iii. Change the "Max Torque Current" to 2000.
    
<p align="center">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/setting_tool_done.png" width="400" height="300">
</p>

13. Press "Save Setting" button on the botton right. 
14. Press the "DISCONNECT" button on the top right, also **power off** the motor, wait about one second, and then power the motor back on. This will allow the motor to memorize the settings.
15. Connect the motor again to check if the settings are correct and memorized by the motor.  
    (**Using set A:**) Ensure that you change the baud rate and ID at the top of the setting tool screen as you have already changed the "RS485 Baudrate" and "Driver ID."  
    (**Using set B:**) The UART baud rate remains unchanged, so continue using the default baud rate of 115200 and change the ID at the top of the setting tool screen as you have already changed the "Driver ID."
16. Now that you have successfully set up the motor, you can return to adjust the "PID Settings" if desired.

### PCB setup
**The "RS485_1" ports are using "Serial5" for communication  
The "RS485_2" ports are using "Serial3" for communication**  
Refer to the datasheet ([Motor_Communication_Control.pdf](datasheet/Motor_Communication_Control.pdf)) for more information.
<p align="center">
 <img src="https://github.com/HaoYuLiu0725/LKM_Motor/blob/main/image/Motor_Communication_Control_PCB.jpg" width="400" height="300">
</p>

## Examples
All examples utilize Teensy 4.0 and the 'Motor Communication Control' PCB in NTU ASR-LAB, employing the 'RS485_1' ports on the PCB.  
("RS485_1" is using Serial5 for communication)  
### Single motor control
[single_motor_control.ino](examples/single_motor_control/single_motor_control.ino)  
The example of single motor control  
This example will control the motor turning from 0 to 60 degree, 60 to 0 degree, 0 to -60 degree, and then from -60 degree turn back to 0 degree.  

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
**Constructor:**  
`LKM_Motor()`  
`LKM_Motor(int id, int reduction_ratio, int serial_port)`  
`LKM_Motor(int id, int reduction_ratio, int serial_port, double Kt)`  

**Serial Initialize:**  
`void Serial_Init()`

**更改馬達baudrate:**  
`void Change_Baudrate(int baudrate)`

(3)讀取電機狀態2命令(0x9C)
`void Read_Motor_State_2();`

(5)電機關機命令(0x80)  
`void Write_Motor_Shutdown()`

(6)電機運行命令(0x88)  
`void Write_Motor_Run()`

(7)電機停止命令(0x81)  
`void Write_Motor_Pause()`

(10)轉矩閉環控制命令(0xA1), current: -32~32  
`void Write_Torque_Current(double current)`

轉矩控制, torque [N*m] = Kt * I  
`void Write_Torque(double torque)`

(11)速度閉環控制命令(0xA2)  
`void Write_Speed(double speed)`

(12)多圈位置閉環控制命令1(0xA3)  
`void Write_Angle_MultiRound(double angle)`

(13)多圈位置閉環控制命令2(0xA4)  
`void Write_Angle_MultiRound(double angle, double max_speed)`

(14)單圈位置閉環控制命令1(0xA5), direction: True -> 順時針 ; False -> 逆時針  
`void Write_Angle_SingleRound(double angle, bool direction);`

(15)單圈位置閉環控制命令2(0xA6), direction: True -> 順時針 ; False -> 逆時針  
`void Write_Angle_SingleRound(double angle, double max_speed, bool direction)`

轉向自動-單圈位置閉環控制命令1(往角度小的方向走)  
`void Write_Angle_SingleRound(double angle)`

轉向自動-單圈位置閉環控制命令2(往角度小的方向走)  
`void Write_Angle_SingleRound(double angle, double max_speed)`

(16)增量位置閉環控制命令1(0xA7)  
`void Write_Angle_Increment(double angle_increment)`

(17)增量位置閉環控制命令2(0xA8)  
`void Write_Angle_Increment(double angle_increment, double max_speed)`

(19)設置馬達零點(0x19)  
`void Set_Motor_Origin()`

(20)讀取多圈角度命令(0x92)  
`void Read_Angle_MultiRound()`

(22)讀取單圈角度命令(0x94)  
`void Read_Angle_SingleRound()`

列印出馬達設定的 id, reduction_ratio, serial_port  
`void Print_Setup_Data()`

列印出馬達回傳的資料: 電機溫度、轉矩電流、電機速度以及編碼器位置  
`void Print_Data()`

列印出馬達回傳的角度  
`void Print_Angle()`

列印出馬達的PID參數  
`void Print_PID_Param()`

設定非讀取資訊的指令是否需要解封包  
`void Set_Need_Receive(bool need_receive)`

設定馬達的轉矩常數Kt  
`void Set_Kt(double Kt)`

## Other
### Setup for Teensy 4.0 on Arduino IDE
[Download and Install Teensy support into the Arduino IDE](https://www.pjrc.com/teensy/td_download.html)

