# LKM_Motor

LK tech's motor control library using Teensy4.0 with Arduino IDE.

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
### Single motor control
[single_motor_control.ino](examples/single_motor_control/single_motor_control.ino)

The example of single motor control
Using Teensy4.0 and "Motor Communication Control" PCB board in NTU ASR-LAB, and use "RS485_1" port on the PCB board
("RS485_1" is using Serial5 for communication)

This example will control the motor turning from 0 to 60 degree, and then from 60 degree turn back to 0 degree.

### Multi motor control

## Library Methods

## Other
### Setup for Teensy 4.0 on Arduino IDE

