# Notice
The nanocar is a commercially avaliable type of ROS robots. This branch is copied from the origin: https://gitee.com/bingda-robot/base_control
- removing unnecessary set up for other robots
- translate important comments and explanation in to English
The package name is also changed to 'vpa_basecontrol' so that it does not require distinguishment in high-level control.

This package send twsit inforation by usart to its low level controller, which is a STM32 MCU.

To use the usart, the hardware for usart shall be enabled and a udev rule must recreated. To create the udev rule, you may check 'scripts/rpi4initsetup.sh'.

# base_control
BingDa Robot move chasssis control package

## usart communication protocols

### usart settings
baudrate:115200,1 stop bit，8 data bit,no parity bit
### general
1. The function code is odd if sent from the raspiberrym and even if from the MCU
2. The 'Length' is defined the length for the whole data package including header to CRC check bits.
3. 'ID' is defined for the MCU, in case of a multi lower controller application
4. 'Reserved' is reserved for now for future development
5. CRC check is one byte with CRC-8/MAXIM
6. Longitude speed unit is m/s，yaw rate unit is rad/s, while angle unit is degree 

| Header | Length | ID | Function code | Data | Reserved | CRC |
| ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| 0x5a | 0x00 | 0x01 | 0x00 | 0xXX...0xXX | 0x00 | 0xXX |

## Reaction for starting & ternimating communication ##
- The lower unit recieves any frames and then considered communication established. The IMU will initialize, in about 2 seconds. It is recommened that the upper communication sending frames in a frequency no lower that 2 Hz
- No frames for 1000ms will be considered the ternimation of communication, the lower unit will brake the motor.

## Function code ##
The part will skip if such sensor is not installed

### 0x01 ###
Upper -> Lower, Length: 6Byte，setting: speed to on x axis *1000(int16_t),  speed on y axis*1000(int16_t) and yaw rate *1000(int16_t)s

|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|
| ---- | ---- | ---- | ---- | ---- | ---- |
|X MSB|X LSB|Y MSB|Y LSB|Z MSB|Z LSB|

Example: 5A 0C 01 01 01 F4 00 00 00 00 00 56 (Moving forward at 0.5 m/s)
### 0x02 ###
Lower -> Upper，Length: 0Byte，reply when only failed to set the speed
### 0x03 ###
Upper -> Lower，Length: 0Byte demanding current speed

Example: 5A 06 01 03 00 DF
### 0x04 ###
Lower -> Upper，Length: 6Byte，replying: speed to on x axis *1000(int16_t), speed on y axis*1000(int16_t) and yaw rate *1000(int16_t)s

|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|
| ---- | ---- | ---- | ---- | ---- | ---- |
|X MSB|X LSB|Y MSB|Y LSB|Z MSB|Z LSB|

### 0x05 ###
Upper -> Lower，Length: 0Byte demanding IMU readings

Example: 5A 06 01 05 00 75
### 0x06 ###
Lower -> Upper，Length: 6Byte replying: IMUPitch*1000(int16_t),Roll*1000(int16_t) and  Yaw*1000(int16_t)

|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|
| ---- | ---- | ---- | ---- | ---- | ---- |
|Pitch MSB|Pitch LSB|Roll MSB|Roll LSB|Yaw MSB|Yaw LSB|

### 0x07 ###
Upper -> Lower，Length: 0Byte demanding Battery readings
Example: 5A 06 01 07 00 E4 

### 0x08 ###
Lower -> Upper，Length: 4Byte replying: battery info, Voltage*1000(uint16_t) and Current*1000(uint16_t)

Data format
|Byte1|Byte2|Byte3|Byte4|
| ---- | ---- | ---- | ---- |
|Voltage MSB|Voltage LSB|Current MSB|Current LSB|

### 0x08 ###
Upper -> Lower，Length: 0Byte demanding Odom readings

Example: 5A 06 01 09 00 38

### 0x0a ###
Lower -> Upper，Length: 6Byte replying: longtitude speed(X)*1000, angle*100, yaw rate (Z)*1000
|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|
| ---- | ---- | ---- | ---- | ---- | ---- |
|X MSB|X LSB|Yaw MSB|Yaw LSB|Z MSB|Z LSB|

### 0x13 ###
Upper -> Lower，Length: 0Byte demanding IMU raw readings
Example: 5A 06 01 13 00 33

### 0x14 ###
Lower -> Upper，Length: 6Byte including
- GyroX*100000(int32_t)
- GyroY*100000(int32_t)
- GyroZ*100000(int32_t)
- AccelX*100000(int32_t)
- AccelY*100000(int32_t)
- AccelZ*100000(int32_t)
- QuatW×10000
- QuatX×10000
- QuatY×10000
- QuatZ×10000
Data format:

| Byte1-4 | Byte5-8 | Byte9-12 | Byte13-16 | Byte17-20 | Byte21-24 | Byte25-26 | Byte27-28 | Byte29-30 | Byte31-32 |
| ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| GyroX | GyroY | GyroZ | AccelX | AccelY | AccelZ | QuatW | QuatX | QuatY | QuatZ |

### 0x15 ###
Upper -> Lower，Length: 6Byte, Ackerman form chassis demand, including speed on X axis*1000(int16_t)+accelX*1000(int16_t),steering angle*1000(int16_t)(in rad)

Note: accelX not in used

|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|
| ---- | ---- | ---- | ---- | ---- | ---- |
|X MSB|X LSB|AX MSB|AX LSB|A MSB|A LSB|

### 0x21 ###
Upper -> Lower，Length: 6Byte, demanding chassis configuration
Example :5a 06 01 21 00 8F

### 0x22 ###
Lower -> Upper，Length: 6Byte, including
- BASE_TYPE (Type of chassis, uint8_t)
- MOTOR_TYPE (Motor type, uint8_t)
- Ratio*10 (Gear reduction ratio, int16_t)
- Diameter*10 (Tire size, int16_t)

|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|
| ---- | ---- | ---- | ---- | ---- | ---- |
|BASE_TYPE|MOTOR_TYPE|ratio MSB|ratio LSB|diameter MSB|diameter LSB|

### 0xf1 ###
Upper -> Lower，Length: 0Byte, demanding lower controller software/hardware version
Example :5a 06 01 f1 00 d7

### 0xf2 ###
Lower -> Upper，Length: 6Byte, hardware version: xx.yy.zz, software version: aa.bb.cc，
|Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|
| ---- | ---- | ---- | ---- | ---- | ---- |
|xx|yy|zz|aa|bb|cc|

### 0xf3 ###
Upper -> Lower，Length: 0Byte, demanding S/N number
Example:5a 06 01 f3 00 46

### 0xf4 ###
Lower -> Upper，Length: 12Byte

### 0xfd ###
Upper -> Lower，Length: 0Byte, demanding reboot
Example: 5a 06 01 fd 00 9a
