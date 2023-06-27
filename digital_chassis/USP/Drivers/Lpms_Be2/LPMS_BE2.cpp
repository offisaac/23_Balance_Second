/**
******************************************************************************
* Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
* @file    LPMS_BE2.cpp
* @author  ZhiRui Zhang 2231625449@qq.com
* @brief   Library for LPMS-BE2.
* @date    2022-11-04
* @version 1.0
* @par Change Log：
* <table>
* <tr><th>Date        <th>Version  <th>Author    		<th>Description
* <tr><td>2022-11-04  <td> 1.0     <td>ZZR  				<td>Creator
* </table>
*
==============================================================================
                    ##### How to use this driver #####
==============================================================================
  @note

                https://git.scutbot.cn/Lu/LPMS_BE2_Library/src/branch/main/LPMS_BE2_Library/Readme.md
  @warning

******************************************************************************
* @attention
*
* if you had modified this file, please make sure your code does not have many
* bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
* through your new brief.
*
* <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
* All rights reserved.</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "LPMS_BE2.h"
#include "internal.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  陀螺仪初始化
 * @note
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Init()
{
//    goto_command_mode();
//    set_imu_id(sensor_id);
//    set_stream_freq(stream_freq);
//    LPMS_BE2_Data_Type(data_type);
//    LPMS_BE2_Data_Range(gyr_range, acc_range, degrad_unit);
//    LPMS_BE2_Filter_Mode(filter_mode, is_auto_cali, filter_mode);
//    goto_stream_mode();
    is_init = true;
}

/**
 * @brief  陀螺仪输出数据类型配置
 * @note
 * @param  DATA_32BIT、DATA_16BIT
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::Data_Type_Config(uint8_t _data_type)
{
    data_type = _data_type;
}

/**
 * @brief  数据流输出频率设置
 * @note
 * @param  FREQ_nHZ，n = 5、10、50、100、250、500
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::Stream_Freq_Config(uint16_t _stream_freq)
{
    stream_freq = _stream_freq;
}
/**
 * @brief  陀螺仪数据量程和单位配置
 * @note
 * @param	 GYR_RANGE_nDPS，n = 125、250、500、1000、2000
 * @param	 ACC_RANGE_nG，n = 2g、4g、8g、16g
 * @param	 DEGREE、RADIAN
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::Data_Range_Config(uint16_t _gyr_range, uint8_t _acc_range, uint8_t _degrad_unit)
{
    gyr_range = _gyr_range;
    acc_range = _acc_range;
    degrad_unit = _degrad_unit;
}

/**
 * @brief  滤波模式及参数配置
 * @note
 * @param	 GYR_ONLY、KALMAN、DCM
 * @param	 AUTO_CALI_ENABLE、AUTO_CALI_DISABLE
 * @param	 OBJECT、HEADING（yaw归零）、ALIGMENT（pitch和roll归零）
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::Filter_Mode_Config(uint8_t _filter_mode, uint8_t _auto_cali, uint8_t _offset_mode)
{
    filter_mode = _filter_mode;
    is_auto_cali = _auto_cali;
    offset_mode = _offset_mode;
}

/**
 * @brief  读取原始数据函数
 * @note
 * @param  传入串口接收数据的地址指针
 * @return 应答成功或接收数据成功则返回1
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::LPMS_BE2_Get_Data(uint8_t *data)
{
    uint16_t data_command = 0;
    if (data[0] == 0x3A && (data[2] << 8 | data[1]) == sensor_id)
    {
        data_command = data[4] << 8 | data[3];
        switch (data_command)
        {
        case REPLY_ACK:
            is_reply = true;
            break;
        case REPLY_NACK:
            is_reply = false;
            break;
        case GET_IMU_DATA:
            if (is_init == true)
            {
                LPMS_BE2_Data_Update(data);
            }
            break;
            // case GET_SENSOR_STATUS:
        // case GET_SENSOR_MODEL:
        // case GET_FIRMWARE_INFO:
        // case GET_SERIAL_NUMBER:
        // case GET_FILTER_VERSION:
        // case GET_IMU_TRANSMIT_DATA:
        // case GET_IMU_ID:
        // case GET_STREAM_FREQ:
        // case GET_DEGRAD_OUTPUT:
        // case GET_ACC_RANGE:
        // case GET_GYR_RANGE:
        // case START_GYR_CALIBRATION:
        // case GET_ENABLE_GYR_AUTOCALIBRATION:
        // case GET_GYR_THRESHOLD:
        // case GET_FILTER_MODE:
        // case GET_UART_BAUDRATE:
        // case GET_UART_FORMAT:
        // case GET_UART_ASCII_CHARACTER:
        // case GET_LPBUS_DATA_PRECISION:
        default:
            break;
        }
        return 1;
    }
    else
    {
    }
    return 0;
}

/**
 * @brief  指令发送打包函数
 * @note   用于把不同指令及数据装进标准数据包
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Command_Pack(CommandPack_Structdef *_command_pack, uint16_t _command, uint32_t _data)
{
    /*装入数据*/
    _command_pack->SensorID_L = sensor_id;
    _command_pack->SensorID_H = sensor_id >> 8;
    _command_pack->Command_L = _command;
    _command_pack->Command_H = _command >> 8;
    _data == 0 ? _command_pack->Length_L = 0, _command_pack->Length_H = 0 : _command_pack->Length_L = 4, _command_pack->Length_H = 0;
    _command_pack->data1 = _data;
    _command_pack->data2 = _data >> 8;
    _command_pack->data3 = _data >> 16;
    _command_pack->data4 = _data >> 24;
    _command_pack->LRC_L = (_command_pack->SensorID_L + _command_pack->SensorID_H + _command_pack->Command_L + _command_pack->Command_H + _command_pack->Length_L + _command_pack->Length_H + _command_pack->data1 + _command_pack->data2 + _command_pack->data3 + _command_pack->data4);
    _command_pack->LRC_H = (_command_pack->SensorID_L + _command_pack->SensorID_H + _command_pack->Command_L + _command_pack->Command_H + _command_pack->Length_L + _command_pack->Length_H + _command_pack->data1 + _command_pack->data2 + _command_pack->data3 + _command_pack->data4) >> 8;
}

/**
 * @brief  发送指令函数函数
 * @note   发送普通指令，例如转换模式，获取数据等
 * @param  command要发送的指令
 * @param  usart_port_num发送的串口号
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Send_Command(CommandPack_Structdef *command_pack, uint8_t usart_port_num)
{
    static USART_COB Command_Cobe;
    uint16_t data_length = 0;
    data_length = (command_pack->Length_H << 8) | (command_pack->Length_L);

    if (data_length == 0x04)
    {
        memcpy(send_data_large, command_pack, sizeof(CommandPack_Structdef)); //完整数据包直接复制
        Command_Cobe.address = send_data_large;
        Command_Cobe.len = sizeof(send_data_large);
    }
    else if (data_length == 0x00)
    {
        memcpy(send_data_small, command_pack, 7);             // 包头数据写入数组
        memcpy(&send_data_small[7], &command_pack->LRC_L, 4); // 包尾数据写入数组
        Command_Cobe.address = send_data_small;
        Command_Cobe.len = sizeof(send_data_small);
    }
    else
    {
    }
    /*发送串口队列*/
    Command_Cobe.port_num = usart_port_num;
    xQueueSend(USART_TxPort, &Command_Cobe, 1);
}

/**
 * @brief  数据联合函数
 * @note   把数据统一单位，转换为float格式
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Data_Convert()
{
    if (data_type == DATA_32BIT)
    {
        unity_data.timestamp = data_32bit.timestamp;
        unity_data.rawAccX = data_32bit.rawAccX;
        unity_data.rawAccY = data_32bit.rawAccY;
        unity_data.rawAccZ = data_32bit.rawAccZ;
        unity_data.calAccX = data_32bit.calAccX;
        unity_data.calAccY = data_32bit.calAccY;
        unity_data.calAccZ = data_32bit.calAccZ;
        unity_data.rawGyroX = data_32bit.rawGyroX;
        unity_data.rawGyroY = data_32bit.rawGyroY;
        unity_data.rawGyroZ = data_32bit.rawGyroZ;
        unity_data.caliGyroX = data_32bit.caliGyroX;
        unity_data.caliGyroY = data_32bit.caliGyroY;
        unity_data.caliGyroZ = data_32bit.caliGyroZ;
        unity_data.aligGyroX = data_32bit.aligGyroX;
        unity_data.aligGyroY = data_32bit.aligGyroY;
        unity_data.aligGyroZ = data_32bit.aligGyroZ;
        unity_data.angularSpeedX = data_32bit.angularSpeedX;
        unity_data.angularSpeedY = data_32bit.angularSpeedY;
        unity_data.angularSpeedZ = data_32bit.angularSpeedZ;
        unity_data.Quaternion1 = data_32bit.Quaternion1;
        unity_data.Quaternion2 = data_32bit.Quaternion2;
        unity_data.Quaternion3 = data_32bit.Quaternion3;
        unity_data.Quaternion4 = data_32bit.Quaternion4;
        unity_data.Euler_Roll = data_32bit.Euler_Roll;
        unity_data.Euler_Pitch = data_32bit.Euler_Pitch;
        unity_data.Euler_Yaw = data_32bit.Euler_Yaw;
        unity_data.linearAccX = data_32bit.linearAccX;
        unity_data.linearAccY = data_32bit.linearAccY;
        unity_data.linearAccZ = data_32bit.linearAccZ;
        unity_data.temperature = data_32bit.temperature;
    }
    else if (data_type == DATA_16BIT)
    {
        unity_data.timestamp = data_16bit.timestamp;
        unity_data.rawAccX = (float)data_16bit.rawAccX / 1000.f;
        unity_data.rawAccY = (float)data_16bit.rawAccY / 1000.f;
        unity_data.rawAccZ = (float)data_16bit.rawAccZ / 1000.f;
        unity_data.calAccX = (float)data_16bit.calAccX / 1000.f;
        unity_data.calAccY = (float)data_16bit.calAccY / 1000.f;
        unity_data.calAccZ = (float)data_16bit.calAccZ / 1000.f;
        unity_data.rawGyroX = (float)data_16bit.rawGyroX / 10.f;
        unity_data.rawGyroY = (float)data_16bit.rawGyroY / 10.f;
        unity_data.rawGyroZ = (float)data_16bit.rawGyroZ / 10.f;
        unity_data.caliGyroX = (float)data_16bit.caliGyroX / 10.f;
        unity_data.caliGyroY = (float)data_16bit.caliGyroY / 10.f;
        unity_data.caliGyroZ = (float)data_16bit.caliGyroZ / 10.f;
        unity_data.aligGyroX = (float)data_16bit.aligGyroX / 10.f;
        unity_data.aligGyroY = (float)data_16bit.aligGyroY / 10.f;
        unity_data.aligGyroZ = (float)data_16bit.aligGyroZ / 10.f;
        unity_data.angularSpeedX = (float)data_16bit.angularSpeedX / 100.f;
        unity_data.angularSpeedY = (float)data_16bit.angularSpeedY / 100.f;
        unity_data.angularSpeedZ = (float)data_16bit.angularSpeedZ / 100.f;
        unity_data.Quaternion1 = (float)data_16bit.Quaternion1 / 10000.f;
        unity_data.Quaternion2 = (float)data_16bit.Quaternion2 / 10000.f;
        unity_data.Quaternion3 = (float)data_16bit.Quaternion3 / 10000.f;
        unity_data.Quaternion4 = (float)data_16bit.Quaternion4 / 10000.f;
        unity_data.Euler_Roll = (float)data_16bit.Euler_Roll / 100.f;
        unity_data.Euler_Pitch = (float)data_16bit.Euler_Pitch / 100.f;
        unity_data.Euler_Yaw = (float)data_16bit.Euler_Yaw / 100.f;
        unity_data.linearAccX = (float)data_16bit.linearAccX / 1000.f;
        unity_data.linearAccY = (float)data_16bit.linearAccY / 1000.f;
        unity_data.linearAccZ = (float)data_16bit.linearAccZ / 1000.f;
        unity_data.temperature = (float)data_16bit.temperature / 100.f;
    }
//		static float last_timeStamp = 0;
//		if(unity_data.timestamp != last_timeStamp)
//		{
//			link_count = 0;
//		}
//		else
//		{
//			link_count++;
//		}
//		last_timeStamp = unity_data.timestamp;
//		if(link_count>100)
//		{
//			goto_stream_mode();
//			link_count = 0;
//		}
//		else
//		{}
}

/**
 * @brief  数据抓取函数
 * @note   用于把不同数据包的数据解包处理
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Data_Update(uint8_t *_data)
{
    uint8_t point = 0;      //用于指向串口数据包不同位置
    uint8_t head_size = 11; //起始数据11个字节
    uint8_t tail_size = 4;  //结尾数据4个字节

    if (data_type == DATA_32BIT)
    {
        /*写入包头数据*/
        memcpy(&data_32bit, _data, head_size);
        point += head_size;
#if RAW_ACC
        memcpy(&data_32bit.rawAccX, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if CAL_ACC
        memcpy(&data_32bit.calAccX, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if RAW_GYRO
        memcpy(&data_32bit.rawGyroX, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if CALI_GYRO
        memcpy(&data_32bit.caliGyroX, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if ALIG_GYRO
        memcpy(&data_32bit.aligGyroX, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if ANGULAR_SPEED
        memcpy(&data_32bit.angularSpeedX, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if QUETERNION
        memcpy(&data_32bit.Quaternion1, &_data[point], 4 * sizeof(float));
        point += 4 * sizeof(float);
#endif
#if EULER
        memcpy(&data_32bit.Euler_Roll, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if LINEAR_ACC
        memcpy(&data_32bit.linearAccX, &_data[point], 3 * sizeof(float));
        point += 3 * sizeof(float);
#endif
#if TEMPERATURE
        memcpy(&data_32bit.temperature, &_data[point], 1 * sizeof(float));
        point += 1 * sizeof(float);
#endif
        /*写入包尾数据*/
        memcpy(&data_32bit.LRC_L, &_data[point], tail_size);
    }
    else if (data_type == DATA_16BIT)
    {
        /*写入包头数据*/
        memcpy(&data_16bit, _data, head_size);
        point += head_size;
#if RAW_ACC
        memcpy(&data_16bit.rawAccX, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if CAL_ACC
        memcpy(&data_16bit.calAccX, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if RAW_GYRO
        memcpy(&data_16bit.rawGyroX, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if CALI_GYRO
        memcpy(&data_16bit.caliGyroX, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if ALIG_GYRO
        memcpy(&data_16bit.aligGyroX, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if ANGULAR_SPEED
        memcpy(&data_16bit.angularSpeedX, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if QUETERNION
        memcpy(&data_16bit.Quaternion1, &_data[point], 4 * sizeof(int16_t));
        point += 4 * sizeof(int16_t);
#endif
#if EULER
        memcpy(&data_16bit.Euler_Roll, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if LINEAR_ACC
        memcpy(&data_16bit.linearAccX, &_data[point], 3 * sizeof(int16_t));
        point += 3 * sizeof(int16_t);
#endif
#if TEMPERATURE
        memcpy(&data_16bit.temperature, &_data[point], 1 * sizeof(int16_t));
        point += 1 * sizeof(int16_t);
#endif
        /*写入包尾数据*/
        memcpy(&data_16bit.LRC_L, &_data[point], tail_size);
    }
}

/**
 * @brief  设置接收数据类型
 * @note
 * @param		DATA_32BIT、DATA_16BIT
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Data_Type(uint32_t _data_type)
{
    set_lpbus_data_precision(_data_type); //改为16bit模式会产生无应答信号，问题在排查
    set_imu_transmit_data();
}

/**
 * @brief  数据量程和单位
 * @note
 * @param	 陀螺仪量程125、250、500、1000、2000
 * @param	 加速度量程2g、4g、8g、16g
 * @param	 0是角度、1是弧度
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Data_Range(uint32_t _gyr_range, uint32_t _acc_range, uint32_t _degrad_unit)
{
    set_gyr_range(_gyr_range);
    set_acc_range(_acc_range);
    set_degrad_output(_degrad_unit); //无应答信号，暂时默认为弧度输出
}

/**
 * @brief  滤波和校准
 * @note
 * @param	 0只有陀螺仪、1加速度和陀螺仪（卡尔曼）、3加速度和陀螺仪（DCM）
 * @param	 0为关闭、1为打开
 * @param	 0是Object、1是Heading（yaw归零）、2是Alignment（pitch和roll归零）
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::LPMS_BE2_Filter_Mode(uint32_t _filter_mode, uint32_t _auto_cali, uint32_t _offset_mode)
{
    set_filter_mode(_filter_mode);
    set_enable_gyr_autocalibration(_auto_cali);
//    set_orientation_offset(_offset_mode);
}

/**
 * @brief  进入命令模式
 * @note
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::goto_command_mode()
{
    LPMS_BE2_Command_Pack(&command, GOTO_COMMAND_MODE, 0);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }                 // 等待回复
    is_reply = false; // 清除标志位
}

/**
 * @brief  进入数据流模式
 * @note
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::goto_stream_mode()
{
    LPMS_BE2_Command_Pack(&command, GOTO_STREAM_MODE, 0);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
}

/**
 * @brief  设置读取数据类型
 * @note   选择需要读取的数据
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::set_imu_transmit_data()
{
    uint32_t data = 0;
    data = RAW_ACC | CAL_ACC << 1 | RAW_GYRO << 3 | CALI_GYRO << 5 | ALIG_GYRO << 7 | ANGULAR_SPEED << 10 | QUETERNION << 11 | EULER << 12 | LINEAR_ACC << 13 | TEMPERATURE << 16;
    LPMS_BE2_Command_Pack(&command, SET_IMU_TRANSMIT_DATA, data);
    LPMS_BE2_Send_Command(&command, usart_num);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }                 // 等待回复
    is_reply = false; // 清除标志位
}

/**
 * @brief  保存当前参数设置
 * @note
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::write_registers()
{
    LPMS_BE2_Command_Pack(&command, WRITE_REGISTERS, 0);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
}

/**
 * @brief  恢复出厂设置
 * @note
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::restore_factory_value()
{
    LPMS_BE2_Command_Pack(&command, RESTORE_FACTORY_VALUE, 0);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
}

/**
 * @brief  设置传感器id
 * @note
 * @param   id编号
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::set_imu_id(uint32_t id)
{
    LPMS_BE2_Command_Pack(&command, SET_IMU_ID, id);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    sensor_id = id;
}

/**
 * @brief  设置数据流频率
 * @note
 * @param   频率值 5、10、50、100、250、500HZ
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_stream_freq(uint32_t freq)
{
    if (freq != 5 && freq != 10 && freq != 50 && freq != 100 && freq != 250 && freq != 500)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_STREAM_FREQ, freq);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  设置输出单位
 * @note
 * @param   0是角度、1是弧度
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_degrad_output(uint32_t unit)
{
    if (unit != 0 && unit != 1)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_DEGRAD_OUTPUT, unit);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  调整传感器坐标系
 * @note    详细涵义见手册
 * @param   0是Object、1是Heading（yaw归零）、2是Alignment（pitch和roll归零）
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_orientation_offset(uint32_t mode)
{
    if (mode != 0 && mode != 1 && mode != 2)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_ORIENTATION_OFFSET, mode);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  重置传感器坐标系
 * @note    回到Object坐标系
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::reset_orientation_offset()
{
    LPMS_BE2_Command_Pack(&command, RESET_ORIENTATION_OFFSET, 0);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
}

/**
 * @brief  设置传感器加速度量程范围
 * @note
 * @param 2g、4g、8g、16g
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_acc_range(uint32_t range)
{
    if (range != 2 && range != 4 && range != 8 && range != 16)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_ACC_RANGE, range);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  设置传感器陀螺仪量程范围
 * @note
 * @param 125、250、500、1000、2000
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_gyr_range(uint32_t range)
{
    if (range != 125 && range != 250 && range != 500 && range != 1000 && range != 2000)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_GYR_RANGE, range);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  打开陀螺仪校准静态误差
 * @note
 * @param
 * @return
 * @retval  None
 */
void LPMS_BE2_Typedef::start_gyr_calibration()
{
    LPMS_BE2_Command_Pack(&command, START_GYR_CALIBRATION, 0);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
}

/**
 * @brief  是否启用自动校准
 * @note
 * @param		0为关闭、1为打开
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_enable_gyr_autocalibration(uint32_t is_enable)
{
    if (is_enable != 0 && is_enable != 1)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, START_GYR_CALIBRATION, is_enable);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  设置传感器滤波模式
 * @note
 * @param   0只有陀螺仪、1加速度和陀螺仪（卡尔曼）、3加速度和陀螺仪（DCM）
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_filter_mode(uint32_t mode)
{
    if (mode != 0 && mode != 1 && mode != 3)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_FILTER_MODE, mode);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  设置串口波特率
 * @note
 * @param   115200、230400、256000、460800、921600
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_uart_baudrate(uint32_t baud)
{
    if (baud != 115200 && baud != 230400 && baud != 256000 && baud != 460800 && baud != 921600)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_UART_BAUDRATE, baud);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/**
 * @brief  设置输出数据格式
 * @note
 * @param   0是16bit、1是32bit
 * @return
 * @retval  None
 */
uint8_t LPMS_BE2_Typedef::set_lpbus_data_precision(uint32_t unit)
{
    if (unit != 0 && unit != 1)
    {
        return 0;
    }
    LPMS_BE2_Command_Pack(&command, SET_LPBUS_DATA_PRECISION, unit);
    while (is_reply == false)
    {
        LPMS_BE2_Send_Command(&command, usart_num);
        vTaskDelay(COMMAND_DELAY);
    }
    is_reply = false; // 清除标志位
    return 1;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
