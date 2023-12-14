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
Byte1   Byte2   Byte3   Byte4   Byte5   Byte6
X MSB   X LSB   Y MSB   Y LSB   Z MSB   Z LSB
Example: 5A 0C 01 01 01 F4 00 00 00 00 00 56 (Moving forward at 0.5 m/s)
### 0x02 ###
Lower -> Upper，Length: 0Byte，reply when only failed to set the speed
### 0x03 ###
Upper -> Lower，Length: 0Byte demanding current speed
Example: 5A 06 01 03 00 DF
### 0x04 ###
Lower -> Upper，Length: 6Byte，replying: speed to on x axis *1000(int16_t), speed on y axis*1000(int16_t) and yaw rate *1000(int16_t)s
Byte1   Byte2   Byte3   Byte4   Byte5   Byte6
X MSB   X LSB   Y MSB   Y LSB   Z MSB   Z LSB
### 0x05 ###
Upper -> Lower，Length: 0Byte demanding IMU readings
Example: 5A 06 01 05 00 75
### 0x06 ###
Lower -> Upper，Length: 6Byte replying: IMUPitch*1000(int16_t),Roll*1000(int16_t) and  Yaw*1000(int16_t)
Byte1   	Byte2   	Byte3   	Byte4   	Byte5		Byte6
Pitch MSB   Pitch LSB   Roll MSB   	Roll LSB   	Yaw MSB   	Yaw LSB

0x07
上位机向下位机查询电池信息，数据长度为0Byte
例:5A 06 01 07 00 E4 

0x08
下位机上报电池信息，数据长度为4Byte，数据为电压Voltage*1000(uint16_t) + 电流Current*1000(uint16_t)
数据格式为：
Byte1   		Byte2   		Byte3   		Byte4 
Voltage MSB		Voltage LSB		Current MSB		Current LSB

0x09
上位机向下位机获取里程计信息
例:5A 06 01 09 00 38

0x0a
下位机上报速度航向信息，数据长度为6Byte，数据为线速度*1000、角度*100、角速度*1000
Byte1   Byte2   Byte3     Byte4     Byte5   Byte6
X MSB   X LSB   Yaw MSB   Yaw LSB   Z MSB   Z LSB

0x11
上位机向下位机获取里程计信息（相比功能码0x09对应的消息，增加了Y轴线速度，为了适应全向移动底盘的需求）
例:5A 06 01 11 00 A2

0x12
下位机上报速度航向信息，数据长度为8Byte，数据为X轴线速度*1000、Y轴线速度*1000、角度*100、角速度*1000，
Byte1   Byte2   Byte3   Byte4    Byte5    Byte6    Byte7   Byte8
X MSB   X LSB   Y MSB   Y LSB    Yaw MSB  Yaw LSB  Z MSB   Z LSB

0x13
上位机向下位机查询IMU原始数据，数据长度为0Byte
5A 06 01 13 00 33

0x14
下位机上报当IMU数据，数据长度为32Byte，数据为GyroX*100000(int32_t)、GyroY*100000(int32_t)、GyroZ*100000(int32_t)、
                                             AccelX*100000(int32_t)、AccelY*100000(int32_t)、AccelZ*100000(int32_t)、
                                             QuatW×10000、QuatX×10000、QuatY×10000、QuatZ×10000
数据格式为：（高位在前，低位在后）
Byte1~4   Byte5~8   Byte9~12   Byte13~16   Byte17~20   Byte21~24   Byte25~26   Byte27~28   Byte29~30   Byte31~32
GyroX     GyroY     GyroZ      AccelX      AccelY      AccelZ      QuatW       QuatX       QuatY       QuatZ

0x15
上位机向下位机发送速度控制指令(阿克曼结构车型)，数据长度为6Byte，数据为X轴方向速度*1000(int16_t) + X轴方向加速度*1000(int16_t) + 转向角度*1000(int16_t)(角度为弧度制)
注：加速度值暂时未使用
Byte1   Byte2   Byte3    Byte4    Byte5   Byte6
X MSB   X LSB   AX MSB   AX LSB   A MSB   A LSB
例:5A 0C 01 15 00 CB 00 00 00 CB 00 74 (底盘以0.2m/s的速度，0.2弧度的转向角向前运动)

0x17
上位机向下位机获取ADC接口的ADC值，ADC采样范围为0~6.6V，ADC值和电压的换算关系为((电压/2)/3.3V)*4095 = ADC值
5A 06 01 17 00 08

0x18
下位机回复上位机ADC值，数据格式为：（高位在前，低位在后）
Byte1~2   Byte3~4   Byte5~6   Byte7~8   Byte9~10   Byt11~12
ADC1      ADC2      ADC3      ADC4      ADC5       ADC6

0x19
上位机向下位机获取超声波测距值，单位为cm
5A 06 01 19 00 D4

0x1a
下位机回复上位机超声波测距值
Byte1       Byte2       Byte3       Byte4
超声波1     超声波2      超声波3      超声波4

0x1b
上位机设置下位机IO口值，数据长度为4Byte，分别对应4个IO口的值，0为低电平，1为高电平
例：5A 0a 01 1B 01 00 00 00 00 93 设置IO1为高电平，IO2、IO3、IO4为低电平

0x1c
下位机回复上位机IO口值
例：5A 0a 01 1C 01 00 00 00 00 16 当前IO1为高电平，IO2、IO3、IO4为低电平

0x1d
上位机设置下位机PWM口输出值，数据长度为8Byte，分别对应4个PWM口的高电平时间，单位为us，PWM周期为20000us
Byte1~2   Byte3~4   Byte5~6   Byte7~8   Byte9~10   Byt11~12
PWM1      PWM2      PWM3      PWM4      PWM5       PWM6
例：5A 0e 01 1D 01 00 00 00 00 00 00 00  00  46 设置PWM1为高电平时间为256us，PWM2、PWM3、PWM4高电平时间为0

0x1e
下位机回复上位机PWM口值
例：5A 0e 01 1E 01 00 00 00 00 00 00 00  00  0D 当前PWM1为高电平时间为256us，PWM2、PWM3、PWM4高电平时间为0

0x1f
上位机设置下位机LED灯条灯颜色和亮度，数据长度为15Byte，分别对应5个LED灯的R、G、B值（值范围为0~255）
例：5A 15 01 1F FF 00 00 00 FF 00 00 80 FF 00 00 00 00 00 00 00 4D 
设置LED1 红色亮度为255，LED2 绿色亮度为255，LED3绿色亮度为128，蓝色亮度为255，其余LED亮度均为0 

0x21
上位机向下位机获取底盘配置信息
例:5a 06 01 21 00 8F

0x22
下位机回复配置信息数据长度为xByte,格式为BASE_TYPE(底盘类型uint8_t) + MOTOR_TYPE(电机型号uint8_t) + ratio*10(电机减速比int16_t) + diameter*10(轮胎直径int16_t)
Byte1       Byte2       Byte3      Byte4      Byte5         Byte6 
BASE_TYPE   MOTOR_TYPE  ratio MSB  ratio LSB  diameter MSB  diameter LSB

0xf1
上位机向下位机查询版本号
例:5a 06 01 f1 00 d7

0xf2
下位机回复版本号，数据长度为6Byte,格式为硬件版本号xx.yy.zz,软件版本号aa.bb.cc，
Byte1   Byte2   Byte3   Byte4   Byte5   Byte6
xx      yy      zz      aa      bb      cc

0xf3
上位机向下位机查询主板SN号
例:5a 06 01 f3 00 46

0xf4
下位机回复版本号，数据长度为12Byte,高位在前，低位在后
Byte1~12 
SN号

0xfd
上位机向下位机发送重启指令,下位机无回复
例:5a 06 01 fd 00 9a
