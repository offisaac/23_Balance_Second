/**
 ******************************************************************************
 * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
 * @file    LPMS_BE2.h
 * @author  ZhiRui Zhang 2231625449@qq.com
 * @brief   Header file of LPMS-BE2.
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _LPMS_BE2_H_
#define _LPMS_BE2_H_

/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
#include "FreeRTOS.h"
#include "queue.h"
// #include "internal.h"

/* Private macros ------------------------------------------------------------*/
/* 数据选择开关 */
#define RAW_ACC 			0		// 原始的加速度数据
#define CAL_ACC 			0		// 校准的加速度数据
#define RAW_GYRO 			0		// 原始的陀螺仪数据
#define CALI_GYRO 		1		// 静止偏差校准后的陀螺仪数据
#define ALIG_GYRO 		1		// 坐标轴校准后的陀螺仪数据
#define ANGULAR_SPEED 0 	// 角速度
#define QUETERNION 		0		// 四元数
#define EULER 				1		// 欧拉角
#define LINEAR_ACC 		1		// 线性加速度
#define TEMPERATURE 	0		// 温度

/* 参数设置宏定义 */
// 数据类型
#define DATA_32BIT 1
#define DATA_16BIT 0
// 陀螺仪量程范围
#define GYR_RANGE_125DPS 125
#define GYR_RANGE_250DPS 250
#define GYR_RANGE_500DPS 500
#define GYR_RANGE_1000DPS 1000
#define GYR_RANGE_2000DPS 2000
// 加速度量程范围
#define ACC_RANGE_2G 2
#define ACC_RANGE_4G 4
#define ACC_RANGE_8G 8
#define ACC_RANGE_16G 16
// 数据输出单位
#define DEGREE 0
#define RADIAN 1
// 滤波模式
#define GYR_ONLY 0
#define KALMAN 1
#define DCM 3
// 使能自动校准
#define AUTO_CALI_ENABLE 1
#define AUTO_CALI_DISABLE 0
// 调整坐标系模式
#define OBJECT 0
#define HEADING 1
#define ALIGMENT 2
// 设置数据流频率
#define FREQ_5HZ 5
#define FREQ_10HZ 10
#define FREQ_50HZ 50
#define FREQ_100HZ 10
#define FREQ_250HZ 250
#define FREQ6_500HZ 500

/*设置初始化指令发送间隔时间*/
#define COMMAND_DELAY 100 // 目前测得较稳定指令间隔(ms)

/*选定输入参数类型*/

#define REPLY_ACK 0x00
#define REPLY_NACK 0x01

#define WRITE_REGISTERS 0x04
#define RESTORE_FACTORY_VALUE 0x05

#define GOTO_COMMAND_MODE 0x06
#define GOTO_STREAM_MODE 0x07

#define GET_SENSOR_STATUS 0x08
#define GET_IMU_DATA 0x09
#define GET_SENSOR_MODEL 0x14
#define GET_FIRMWARE_INFO 0x15
#define GET_SERIAL_NUMBER 0x16
#define GET_FILTER_VERSION 0x17
#define SET_IMU_TRANSMIT_DATA 0x1E
#define GET_IMU_TRANSMIT_DATA 0x1F

#define SET_IMU_ID 0x20
#define GET_IMU_ID 0x21

#define SET_STREAM_FREQ 0x22
#define GET_STREAM_FREQ 0x23

#define SET_DEGRAD_OUTPUT 0x24
#define GET_DEGRAD_OUTPUT 0x25

#define SET_ORIENTATION_OFFSET 0x26
#define RESET_ORIENTATION_OFFSET 0x27

#define SET_ACC_RANGE 0x32
#define GET_ACC_RANGE 0x33

#define SET_GYR_RANGE 0x3C
#define GET_GYR_RANGE 0x3D

#define START_GYR_CALIBRATION 0x3E
#define SET_ENABLE_GYR_AUTOCALIBRATION 0x40
#define GET_ENABLE_GYR_AUTOCALIBRATION 0x41

#define SET_GYR_THRESHOLD 0x42
#define GET_GYR_THRESHOLD 0x43

#define SET_FILTER_MODE 0x5A
#define GET_FILTER_MODE 0x5B

#define SET_UART_BAUDRATE 0x82
#define GET_UART_BAUDRATE 0x83
#define SET_UART_FORMAT 0x84
#define GET_UART_FORMAT 0x85
#define SET_UART_ASCII_CHARACTER 0x86
#define GET_UART_ASCII_CHARACTER 0x87

#define SET_LPBUS_DATA_PRECISION 0x88
#define GET_LPBUS_DATA_PRECISION 0x89
#define SET_TIMESTAMP 0x98

/* Private type --------------------------------------------------------------*/
/*32位数据结构体*/
#pragma pack(1) // 结构体按一个字节对齐
typedef struct _DataPack32bit_Structdef
{
	uint8_t head;		// 包头
	uint8_t SensorID_L; // 低位传感器ID
	uint8_t SensorID_H; // 高位传感器ID
	uint8_t InsID_L;	// 低位指令号
	uint8_t InsID_H;	// 高位指令号
	uint8_t Lenth_L;	// 低位数据长度
	uint8_t Lenth_H;	// 高位数据长度
	/*时间戳*/
	uint32_t timestamp;
	/*原始的加速度计数据 (g)*/
	float rawAccX;
	float rawAccY;
	float rawAccZ;
	/*校准后的加速度计数据 (g)*/
	float calAccX;
	float calAccY;
	float calAccZ;
	/*原始的陀螺仪数据 (dps (默认) / rad/s)*/
	float rawGyroX;
	float rawGyroY;
	float rawGyroZ;
	/*静止偏差校准后的陀螺仪数据 (dps (默认) / rad/s)*/
	float caliGyroX;
	float caliGyroY;
	float caliGyroZ;
	/*坐标轴校准后的陀螺仪数据 (dps (默认) / rad/s)*/
	float aligGyroX;
	float aligGyroY;
	float aligGyroZ;
	/*角速度(rad/s)*/
	float angularSpeedX;
	float angularSpeedY;
	float angularSpeedZ;
	/*四元数 (归一化单位*/
	float Quaternion1;
	float Quaternion2;
	float Quaternion3;
	float Quaternion4;
	/*欧拉角数据 (degree (默认) / r)*/
	float Euler_Roll;
	float Euler_Pitch;
	float Euler_Yaw;
	/*线性加速度(g)*/
	float linearAccX;
	float linearAccY;
	float linearAccZ;
	/*温度*/
	float temperature;
	/*LRC校验*/
	uint8_t LRC_L;
	uint8_t LRC_H;
	/*包尾*/
	uint8_t pack_tail_L;
	uint8_t pack_tail_H;
} DataPack32bit_Structdef;
#pragma pack()

/*16位数据结构体*/
#pragma pack(1)
typedef struct _DataPack16bit_Structdef
{
	uint8_t head;		// 包头
	uint8_t SensorID_L; // 低位传感器ID
	uint8_t SensorID_H; // 高位传感器ID
	uint8_t InsID_L;	// 低位指令号
	uint8_t InsID_H;	// 高位指令号
	uint8_t Lenth_L;	// 低位数据长度
	uint8_t Lenth_H;	// 高位数据长度
	/*时间戳*/
	uint32_t timestamp;
	/*原始的加速度计数据 (g)*/
	int16_t rawAccX;
	int16_t rawAccY;
	int16_t rawAccZ;
	/*校准后的加速度计数据 (g)*/
	int16_t calAccX;
	int16_t calAccY;
	int16_t calAccZ;
	/*原始的陀螺仪数据 (dps (默认) / rad/s)*/
	int16_t rawGyroX;
	int16_t rawGyroY;
	int16_t rawGyroZ;
	/*静止偏差校准后的陀螺仪数据 (dps (默认) / rad/s)*/
	int16_t caliGyroX;
	int16_t caliGyroY;
	int16_t caliGyroZ;
	/*坐标轴校准后的陀螺仪数据 (dps (默认) / rad/s)*/
	int16_t aligGyroX;
	int16_t aligGyroY;
	int16_t aligGyroZ;
	/*角速度(rad/s)*/
	int16_t angularSpeedX;
	int16_t angularSpeedY;
	int16_t angularSpeedZ;
	/*四元数 (归一化单位*/
	int16_t Quaternion1;
	int16_t Quaternion2;
	int16_t Quaternion3;
	int16_t Quaternion4;
	/*欧拉角数据 (degree (默认) / r)*/
	int16_t Euler_Roll;
	int16_t Euler_Pitch;
	int16_t Euler_Yaw;
	/*线性加速度(g)*/
	int16_t linearAccX;
	int16_t linearAccY;
	int16_t linearAccZ;
	/*温度*/
	int16_t temperature;
	/*LRC校验*/
	uint8_t LRC_L;
	uint8_t LRC_H;
	/*包尾*/
	uint8_t pack_tail_L;
	uint8_t pack_tail_H;
} DataPack16bit_Structdef;
#pragma pack()

/*统一单位数据包*/
typedef struct _Unity_Data_Structdef
{
	/*时间戳*/
	uint32_t timestamp;
	/*原始的加速度计数据 (g)*/
	float rawAccX;
	float rawAccY;
	float rawAccZ;
	/*校准后的加速度计数据 (g)*/
	float calAccX;
	float calAccY;
	float calAccZ;
	/*原始的陀螺仪数据 (dps (默认) / rad/s)*/
	float rawGyroX;
	float rawGyroY;
	float rawGyroZ;
	/*静止偏差校准后的陀螺仪数据 (dps (默认) / rad/s)*/
	float caliGyroX;
	float caliGyroY;
	float caliGyroZ;
	/*坐标轴校准后的陀螺仪数据 (dps (默认) / rad/s)*/
	float aligGyroX;
	float aligGyroY;
	float aligGyroZ;
	/*角速度(rad/s)*/
	float angularSpeedX;
	float angularSpeedY;
	float angularSpeedZ;
	/*四元数 (归一化单位*/
	float Quaternion1;
	float Quaternion2;
	float Quaternion3;
	float Quaternion4;
	/*欧拉角数据 (degree (默认) / r)*/
	float Euler_Roll;
	float Euler_Pitch;
	float Euler_Yaw;
	/*线性加速度(g)*/
	float linearAccX;
	float linearAccY;
	float linearAccZ;
	/*温度*/
	float temperature;
} Unity_Data_Structdef;

/*发送指令标准格式包*/
#pragma pack(1)
typedef struct _CommandPack_Structdef
{
	uint8_t head = 0x3A;
	uint8_t SensorID_L;		 // 低位传感器ID
	uint8_t SensorID_H;		 // 高位传感器ID
	uint8_t Command_L;		 // 低位指令
	uint8_t Command_H;		 // 高位指令
	uint8_t Length_L = 0x00; // 低位长度
	uint8_t Length_H = 0x00; // 高位长度
	/*数据*/
	uint8_t data1 = 0;
	uint8_t data2 = 0;
	uint8_t data3 = 0;
	uint8_t data4 = 0;
	/*LRC校验*/
	uint8_t LRC_L;
	uint8_t LRC_H;
	/*包尾*/
	uint8_t pack_tail_L = 0x0D;
	uint8_t pack_tail_H = 0x0A;
} CommandPack_Structdef;
#pragma pack()

class LPMS_BE2_Typedef
{
public:
	LPMS_BE2_Typedef(uint8_t _usart_num, uint8_t _sensor_id = 0x01) : usart_num(_usart_num), sensor_id(_sensor_id)
	{
	}
	void LPMS_BE2_Init();																	 // 初始化函数
	uint8_t LPMS_BE2_Get_Data(uint8_t *data);												 // 读取原始数据函数
	void LPMS_BE2_Data_Convert();															 // 数据转换函数
	void Data_Type_Config(uint8_t _data_type);												 // 陀螺仪数据输出类型设置函数
	void Stream_Freq_Config(uint16_t _stream_freq);											 // 陀螺仪数据流频率设置函数
	void Data_Range_Config(uint16_t _gyr_range, uint8_t _acc_range, uint8_t _degrad_unit);	 // 陀螺仪数据量程设置函数
	void Filter_Mode_Config(uint8_t _filter_mode, uint8_t _auto_cali, uint8_t _offset_mode); // 陀螺仪滤波修正设置函数

	bool is_init = false;										  // 陀螺仪是否成功初始化
	const Unity_Data_Structdef &get_data() { return unity_data; } // 返回一个联合数据类型的常量引用（不能被外部更改）

	int16_t link_count = 0;

private:
	bool is_reply = false;					 // 是否收到应答信号
	uint8_t usart_num;						 // 使用串口串口号
	uint8_t data_type = DATA_32BIT;			 // 接收数据类型
	uint8_t sensor_id = 1;					 // 传感器id号
	uint16_t stream_freq = FREQ6_500HZ;		 // 数据流频率
	uint8_t degrad_unit = RADIAN;			 // 输出数据单位
	uint8_t orientation_offset = ALIGMENT;	 // 坐标系调整模式
	uint8_t acc_range = ACC_RANGE_4G;		 // 加速度量程
	uint16_t gyr_range = GYR_RANGE_2000DPS;	 // 陀螺仪量程
	uint8_t filter_mode = KALMAN;			 // 滤波模式
	uint8_t offset_mode = ALIGMENT;			 // 调整传感器坐标系模式
	uint32_t baudrate = 230400;				 // 波特率
	uint8_t is_auto_cali = AUTO_CALI_ENABLE; // 是否开启自动校准

	uint8_t send_data_large[15]; // 发送指令数据包
	uint8_t send_data_small[11];

	DataPack32bit_Structdef data_32bit;
	DataPack16bit_Structdef data_16bit;
	CommandPack_Structdef command;
	Unity_Data_Structdef unity_data;

	// 收发数据函数
	void LPMS_BE2_Send_Command(CommandPack_Structdef *command_pack, uint8_t usart_port_num);				 // 发送普通指令函数
	void LPMS_BE2_Data_Update(uint8_t *_data);																 // 数据包抓取函数
	void LPMS_BE2_Command_Pack(CommandPack_Structdef *_command_pack, uint16_t _command, uint32_t _data = 0); // 指令发送打包函数
	// 陀螺仪初始化配置函数
	void LPMS_BE2_Data_Type(uint32_t _data_type);
	void LPMS_BE2_Data_Range(uint32_t _gyr_range, uint32_t _acc_range, uint32_t _degrad_unit);
	void LPMS_BE2_Filter_Mode(uint32_t _filter_mode, uint32_t _auto_cali, uint32_t _offset_mode);
	// 陀螺仪指令发送函数
	void set_imu_transmit_data();
	void goto_command_mode();
	void goto_stream_mode();
	void write_registers();
	void restore_factory_value();
	void set_imu_id(uint32_t id);
	void reset_orientation_offset();
	void start_gyr_calibration();
	uint8_t set_stream_freq(uint32_t freq);
	uint8_t set_degrad_output(uint32_t unit);
	uint8_t set_orientation_offset(uint32_t mode);
	uint8_t set_acc_range(uint32_t range);
	uint8_t set_gyr_range(uint32_t range);
	uint8_t set_enable_gyr_autocalibration(uint32_t is_enable);
	uint8_t set_filter_mode(uint32_t mode);
	uint8_t set_uart_baudrate(uint32_t baud);
	uint8_t set_lpbus_data_precision(uint32_t unit);
};

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
