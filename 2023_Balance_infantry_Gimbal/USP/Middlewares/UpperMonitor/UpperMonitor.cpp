/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    UpperMonitor.c
  * @author  LiangHong Lin(林亮洪)
  * @brief   Code for Upper monitor supported by Mr.Lin in STM32F4.
  * @date    Unkown.
  * @version 1.0
  * @par Change Log：
  * <table
  * <tr><th>Date        <th>Version  <th>Author    		  <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>LiangHong Lin  <td>Creator
  * </table>2019-11-06  <td> 1.1     <td>Mentos Seetoo  <td>Add return valie for
  *                                                         `RecHandle()`
  *
  ==============================================================================
                      ##### How to use this driver #####
  ==============================================================================
    @note
      -# 在下面指定区域`extern`需要观察或者修改的变量。 \n
      -# 如果要观察变量，按格式改`UpperMonitor_Sent_Choose()`或 \n
         如果要修改变量，按格式改`PARAMETER_MODIFICATION()`。
      -# 调用`RecHandle()`处理上位机发过来的所有数据包(通常在串口中断函数中直接调用)
      -# 调用`Sent_Contorl()`发送数据给上位机。
    @warning
      -# 用户需要根据硬件连接修改`Sent_Contorl()`里用于串口发送的函数。

  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have any
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */

/***********************上位机调参使用***********************/
/* 在这里extern需要使用的变量和需要包含的头文件 */

/***********************上位机调参使用***********************/

/* Includes ------------------------------------------------------------------*/
#include <Middlewares/UpperMonitor/UpperMonitor.h>
#include "internal.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern float debug_pitch_angle;
extern float debug_yaw_angle;
extern float yaw_out;
extern float av_pidout;
extern float blpf_out;
extern uint8_t cnt_y;
extern float debug_ff_t;
extern uint32_t bullect_speed_t;
extern uint32_t turn_plate_t;
extern uint32_t vision_can_shoot_t;


/**
 * @brief
 */
#define Sent_Data_Num 9
uint8_t On_Off_flag;
type_change Sent_data_type[Sent_Data_Num + 2];                    //传输数据共用体
uint8_t USART0_Sent_Choose_Data[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8}; //串口选择发送的数据标志

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void UpperMonitor_Sent_Set(float *data);
void UpperMonitor_Sent_Choose(float *data);
float PARAMETER_Change_float(uint8_t *PARAMETER);
void PARAMETER_MODIFICATION(uint8_t *PARAMETER);
void MODE_MODIFICATION(uint8_t *PARAMETER);
/* function prototypes -------------------------------------------------------*/
/**
 * @brief  发送数据函数
 * @param  None
 * @return None
 */
void Sent_Contorl(UART_HandleTypeDef *huart_x)
{
  float temp[Sent_Data_Num];
  UpperMonitor_Sent_Choose(temp); //选择要传输的数据
  UpperMonitor_Sent_Set(temp);    //发送数据转换格式
  HAL_UART_Transmit_DMA(huart_x, (uint8_t *)Sent_data_type + 3, 39);
}

/**
 * @brief  串口发送参数选择函数(要观看的曲线),用于选择需要传输的数据
 * @param  data:需要传输的数组指针
 * @return None.
 */
void UpperMonitor_Sent_Choose(float *data)
{
  uint8_t i;
  for (i = 0; i < Sent_Data_Num; i++)
  {
    switch (USART0_Sent_Choose_Data[i])
    {
      /* 以下部分用于观察参数曲线 */
      /*debug，云台测试*/
      case 0: data[i]= infantry.gimbal.pitch_angleloop.Target;
          break;
      case 1: data[i]= infantry.gimbal.pitch_angleloop.Current;
          break;
      case 2: data[i]= infantry.gimbal.yaw_angleloop.Target;
          break;
      case 3: data[i]= infantry.gimbal.yaw_angleloop.Current;
          break;
      case 4: data[i]= (float)infantry.booster.left_fri_speedloop.Current;//infantry.board_com.rx_pack1.heat_limit;
          break;
      case 5: data[i]= (float)infantry.booster.right_fri_speedloop.Current;//infantry.board_com.rx_pack1.booster_heat;
          break;
      case 6: data[i]= infantry.booster.bulletSpeed;
      		break;
      case 7: data[i]= infantry.pc_vision.PackFromVisionUnion.PackFromVision.yawData;
      		break;
      case 8: data[i]= infantry.pc_vision.PackFromVisionUnion.PackFromVision.pitchData;
      		break;
//			case 0:
//				data[i] = (float)vision_can_shoot_t * 0.001;
//			break;
//			case 1:
//				data[i] = (float)bullect_speed_t * 0.001;
//			break;
//			case 2:
//				data[i] = (float)turn_plate_t * 0.001;
//			break;

      // case 0: data[i]= infantry.gimbal.pitch_angleloop.Target;
      //     break;
      // case 1: data[i]= infantry.gimbal.pitch_angleloop.Current;
      //     break;
      // case 2: data[i]= infantry.gimbal.yaw_angleloop.Target;
      //     break;
      // case 3: data[i]= infantry.gimbal.yaw_angleloop.Current;
      //     break;
      // case 4: data[i]= infantry.gimbal.yaw_currentloop.Target;//infantry.board_com.rx_pack1.heat_limit;
      //     break;
      // case 5: data[i]= infantry.gimbal.yaw_currentloop.Current;//infantry.board_com.rx_pack1.booster_heat;
      //     break;
      // case 6: data[i]= infantry.gimbal.yaw_target;
      // 		break;
      // case 7: data[i]= infantry.gimbal.angle_feedback_yaw;
      // 		break;
      // case 8: data[i]= debug_ff_t;
      // 		break;

//      			case 0: data[i]= infantry.gimbal.pitch_angleloop.Target;
//               break;
//           case 1: data[i]= infantry.gimbal.pitch_angleloop.Current;
//               break;
//           case 2: data[i]= infantry.gimbal.pitch_angleloop.P_Term;
//               break;
//      			case 3: data[i]= infantry.gimbal.pitch_angleloop.I_Term;
//               break;
//      			case 4: data[i]= infantry.gimbal.pitch_currentloop.Target;//infantry.board_com.rx_pack1.heat_limit;
//               break;
//      			case 5: data[i]= infantry.gimbal.pitch_currentloop.Current;//infantry.board_com.rx_pack1.booster_heat;
//               break;
//      			case 6: data[i]= infantry.gimbal.pitch_currentloop.Out;
//      					break;
//      			case 7: data[i]= infantry.gimbal.pitchMotor.givenCurrent;
//      					break;
//           case 8: data[i]= infantry.gimbal.pitchMotor.Out;
//      					break;
								
						// case 0: data[i]= infantry.gimbal.yaw_angleloop.Target;
            //     break;
            // case 1: data[i]= infantry.gimbal.yaw_angleloop.Current;
            //     break;
            // case 2: data[i]= infantry.gimbal.yaw_angleloop.P_Term;
            //     break;
       			// case 3: data[i]= infantry.gimbal.yaw_angleloop.I_Term;
            //     break;
       			// case 4: data[i]= infantry.gimbal.yaw_currentloop.Target;//infantry.board_com.rx_pack1.heat_limit;
            //     break;
       			// case 5: data[i]= infantry.gimbal.yaw_currentloop.Current;//infantry.board_com.rx_pack1.booster_heat;
            //     break;
       			// case 6: data[i]= infantry.gimbal.yaw_currentloop.Out;
       			// 		break;
       			// case 7: data[i]= infantry.gimbal.yawMotor.givenCurrent;
       			// 		break;
            // case 8: data[i]= infantry.gimbal.yawMotor.Out;
       			// 		break;

      //    case 0:
      //      data[i] = infantry.gimbal.yaw_controller.currentLoop.Target;
      //      break;
      //    case 1:
      //      data[i] = infantry.gimbal.yaw_controller.currentLoop.Current;
      //      break;
      //    case 2:
      //      data[i] = yaw_out;
      //      break;
      //    case 3:
      //      data[i] = infantry.gimbal.yawMotor.Out;
      //      break;
      //    case 4:
      //      data[i] = infantry.gimbal.yaw_controller.speedLoop.Target; // infantry.board_com.rx_pack1.heat_limit;
      //      break;
      //    case 5:
      //      data[i] = infantry.gimbal.yaw_controller.speedLoop.Current; // infantry.board_com.rx_pack1.booster_heat;
      //      break;
      //    case 6:
      //      data[i] = infantry.gimbal.yaw_controller.angleLoop.Target;
      //      break;
      //    case 7:
      //      data[i] = infantry.gimbal.yaw_controller.angleLoop.Current;
      //      break;
      //    case 8:
      //      data[i] = infantry.gimbal.yaw_controller.currentLoop.Out;
      //      break;
      /*debug,小发射测试*/
      //  case 0: data[i]= infantry.booster.left_fri_speedloop.Target;
      //      break;
      //  case 1: data[i]= infantry.booster.left_fri_speedloop.Current;
      //      break;
      //  case 2: data[i]= infantry.booster.right_fri_speedloop.Target;
      //      break;
      // 	case 3: data[i]= infantry.booster.right_fri_speedloop.Current;
      // 			break;
      // 	case 4: data[i]= infantry.booster.bulletSpeed;
      // 			break;
      // 	case 5: data[i]= (float)infantry.pc_vision.count;
      // 			break;
      // 	case 6: data[i]= (float)infantry.pc_vision.shoot_mode;
      // 			break;
      // 	case 7: data[i]= infantry.booster.turnplate_angleloop.Target;
      // 			break;
      //   case 8:
      //       data[i] = infantry.booster.turnplate_angleloop.Current;
      //       break;
      /*debug,视觉联调*/
      //		case 0: data[i]= infantry.board_com.tx_pack.chassis_speed_x;//infantry.gimbal.pitch_angleloop.Current;
      //				break;
      //		case 1: data[i]= infantry.gimbal.pitch_angleloop.Target;
      //				break;
      //		case 2: data[i]= infantry.gimbal.pitch_speedloop.Current;
      //				break;
      //		case 3: data[i]= infantry.gimbal.pitch_speedloop.Target;
      //				break;
      //		case 4: data[i]= infantry.gimbal.yaw_angleloop.Current;
      //				break;
      //		case 5: data[i]= infantry.gimbal.yaw_angleloop.Target;
      //				break;
      //		case 6: data[i]= infantry.gimbal.yaw_speedloop.Current;
      //				break;
      //		case 7: data[i]= infantry.gimbal.yaw_speedloop.Target;
      //				break;
      //		case 8: data[i]= infantry.pc_vision.PackFromVisionUnion.PackFromVision.pitchData;
      //				break;
      //		case 9: data[i]= infantry.board_com.tx_pack.chassis_speed_x;//infantry.pc_vision.PackFromVisionUnion.PackFromVision.yawData;
      //				break;
      /*debug,时序核对*/
      //      case 0: data[i]= infantry.gimbal.Get_Angular_Velocity_Pitch();
      //          break;
      //      case 1: data[i]= infantry.gimbal.Get_Angular_Velocity_Yaw();
      //          break;
      //      case 2: data[i]= infantry.pc_vision.PackFromVisionUnion.PackFromVision.pitchData;
      //          break;
      //			case 3: data[i]= infantry.pc_vision.PackFromVisionUnion.PackFromVision.yawData;
      //          break;
      //			case 4: data[i]= infantry.gimbal.yaw_angleloop.Current;
      //					break;
      /*debug，转向环测试*/
      //           case 0: data[i]= infantry.chassisCTRL.chassis_yawAngle.Target;
      //               break;
      //           case 1: data[i]= infantry.chassisCTRL.chassis_yawAngle.Current;
      //               break;
      //           case 2: data[i]= infantry.chassisCTRL.chassis_yawAngle.Out;
      //               break;
      //           case 3: data[i]= infantry.gimbal.Get_YawTotal();
      //               break;
      //           case 4: data[i]= infantry.gimbal.Get_PitchCurrent();
      //               break;
      //           case 5: data[i]= infantry.gimbal.yaw_target;
      //               break;
      //           case 6: data[i]= infantry.chassisCTRL.chassis_yawAngle.Error;
      //               break;
      //           case 7:
      //               data[i] = infantry.chassisCTRL.rotationState;
      //               break;
      /*前馈测试*/
      //      case 0: data[i]= infantry.gimbal.Get_PitchCurrent();
      //          break;
      //      case 1: data[i]= infantry.gimbal.pitchMotor.Out;
      //          break;

      //  case 0:
      //    data[i] = infantry.gimbal.pitchMotor.Out;
      //    break;
      //  case 1:
      //    data[i] = infantry.gimbal.pitchMotor.getSpeed();
      //    break;
      //  case 2:
      //    data[i] = infantry.gimbal.pitch_currentloop.Target;
      //    break;
      //  case 3:
      //  	data[i] = infantry.gimbal.pitch_controller.angleLoop.Target;
      //    break;
      //  case 4:
      //    data[i] = infantry.gimbal.pitch_angleloop.Target;
      //    break;
      //  case 5:
      // 	 data[i] = infantry.gimbal.pitch_controller.angleLoop.Current;
      // 	 break;
      //  case 6:
      // 	 data[i] = infantry.gimbal.pitch_controller.speedLoop.Target;
      // 	 break;
      //  case 7:
      // 	 data[i] = infantry.gimbal.pitchMotor.givenCurrent - (116.2253f * infantry.gimbal.current_pitch - 994.4664f + 231.1759f + 1750.f);
      // 	 break;
      //  case 8:
      // 	 data[i] = infantry.gimbal.pitch_controller.speedLoop.Current;
      // 		break;
      //	case 0:
      //       data[i] = infantry.gimbal.yaw_controller.angleLoop.Target;
      //       break;
      //     case 1:
      //       data[i] = infantry.gimbal.yaw_controller.angleLoop.Current;
      //       break;
      //     case 2:
      //       data[i] = infantry.gimbal.yaw_controller.speedLoop.Target;
      //       break;
      //     case 3:
      //     	data[i] = infantry.gimbal.yaw_controller.speedLoop.Current;
      //       break;
      //     case 4:
      //       data[i] = infantry.gimbal.yaw_controller.currentLoop.Target;
      //       break;
      //		 case 5:
      //			 data[i] = infantry.gimbal.yaw_controller.currentLoop.Current;
      //			 break;
      //		 case 6:
      //			 data[i] = infantry.gimbal.yawMotor.givenCurrent;
      //			 break;
      //		 case 7:
      //			 data[i] = infantry.gimbal.yawMotor.getSpeed();
      //			 break;
      //		 case 8:
      //			 data[i] = infantry.gimbal.pitch_speedloop.Current;
      //			 break;
//   case 0:
//     data[i] = infantry.gimbal.yaw_controller.angleLoop.Target;
//     break;
//   case 1:
//     data[i] = infantry.gimbal.yaw_controller.angleLoop.Current;
//     break;
//   case 2:
//     data[i] = infantry.gimbal.yaw_controller.speedLoop.Target;
//     break;
//   case 3:
//     data[i] = infantry.gimbal.yaw_controller.speedLoop.Current;
//     break;
//   case 4:
//     data[i] = infantry.gimbal.pitch_controller.angleLoop.Target;
//     break;
//   case 5:
//     data[i] = infantry.gimbal.pitch_controller.angleLoop.Current;
//     break;
//   case 6:
//     data[i] = infantry.gimbal.pitch_controller.speedLoop.Target;
//     break;
//   case 7:
//     data[i] = infantry.gimbal.yaw_controller.angle_feedback;
//     break;
//   case 8:
//     data[i] = infantry.gimbal.pitch_controller.angle_feedback;
//			break;
//			case 0:
//				data[i] = infantry.gimbal.yaw_angleloop.Target;
//				break;
//			case 1:
//				data[i] = infantry.gimbal.yaw_angleloop.Current;
//				break;
//			case 2:
//				data[i] = infantry.gimbal.yaw_angleloop.Out;
//				break;
//			case 3:
//				data[i] = infantry.gimbal.yaw_currentloop.Target;
//				break;
//			case 4:
//				data[i] = infantry.gimbal.yaw_currentloop.Current;
//				break;
//			case 5:
//				data[i] = infantry.gimbal.yaw_currentloop.Out;
//				break;
//			case 6:
//				data[i] = infantry.gimbal.yaw_target;
//				break;
    default:
      break;
      /* 以上部分用于观察参数曲线 */
    }
  }
}

/**
 * @brief  上位机参数修改函数（要调的参数）
 * @param  PARAMETER：指令数组指针，用于读取指令
 * @return None.
 */
void PARAMETER_MODIFICATION(uint8_t *PARAMETER)
{
  switch (PARAMETER[0])
  {
  /* 以下部分用于修改参数内容 */
  /*debug，小发射测试*/
  case 0x00:
    infantry.booster.adaptive_fri_wheel_rpm = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x01:
    infantry.booster.turnplate_frq = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x02:
    infantry.booster.heat_offset = PARAMETER_Change_float(PARAMETER + 1);
    break;
  default:
    break;
    /* 以上部分用于修改参数内容 */
  }
}

/**
 * @brief  串口发送设置函数,用于设置DMA串口的数据
 * @param  data:需要传输的数组指针
 * @return None.
 */
void UpperMonitor_Sent_Set(float *data)
{
  uint8_t j;
  Sent_data_type[0].change_u8[3] = 0xfd;  //发送数据头
  for (j = 1; j < Sent_Data_Num + 1; j++) //数据体
  {
    Sent_data_type[j].change_float = data[j - 1];
  }
  Sent_data_type[Sent_Data_Num + 1].change_u8[0] = Sent_Data_Num; //数据尾
  Sent_data_type[Sent_Data_Num + 1].change_u8[1] = 0xfe;          //校验位
}

/**
 * @brief  上位机参数转变成浮点数函数
 * @param  PARAMETER：指令数组指针，用于读取指令
 * @return None.
 */
float PARAMETER_Change_float(uint8_t *PARAMETER)
{
  uint8_t i = 0;
  union type_change Sent_data_temp; //传输数据共用体
  for (i = 0; i < 4; i++)
  {
    Sent_data_temp.change_u8[i] = PARAMETER[3 - i]; //转换成共用体数据类型
  }
  return Sent_data_temp.change_float; //返回共用体转化后的数据
}

/**
 * @brief  上位机参数修改函数
 * @param  PARAMETER： 指令数组指针，用于读取指令
 * @return None.
 */
void MODE_MODIFICATION(uint8_t *PARAMETER)
{
  switch (PARAMETER[0])
  {
  case 0x00:
    USART0_Sent_Choose_Data[0] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x01:
    USART0_Sent_Choose_Data[1] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x02:
    USART0_Sent_Choose_Data[2] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x03:
    USART0_Sent_Choose_Data[3] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x04:
    USART0_Sent_Choose_Data[4] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x05:
    USART0_Sent_Choose_Data[5] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x06:
    USART0_Sent_Choose_Data[6] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x07:
    USART0_Sent_Choose_Data[7] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  case 0x08:
    USART0_Sent_Choose_Data[8] = PARAMETER_Change_float(PARAMETER + 1);
    break;
  default:
    break;
  }
}

uint8_t USART_Interrupt_flag = 0xff; //串口中断标志位
uint8_t USART_Get_Num_Flag = 0;      //串口数据获取标志
uint8_t USART_receive[5] = {0};      //串口接收缓存数组
int len = 0;
/**
* @brief  串口接收解析函数
* @param  data_buf：接收到的数据指针
          length  ：数据长度
* @return No meaning.
*/
uint32_t RecHandle(uint8_t *data_buf, uint16_t length)
{
  uint8_t Temp = 0;
  len = length;
  for (int i = 0; i < length; i++)
  {
    Temp = data_buf[i];
    switch (USART_Interrupt_flag)
    {
    case 0xff:                       // USART0_Interrupt_flag==0xff时为等待模式，等待指令头输入
      if (Temp == 0xf0)              //指令头，识别上位机发送了修改指令
        USART_Interrupt_flag = 0xf0; //下一个指令将进入模式选择模式
      break;
    case 0xf0:          //进入模式选择
      if (Temp == 0x00) //修改参数
      {
        USART_Interrupt_flag = 0x00; //进入参数修改模式
        USART_Get_Num_Flag = 0;
      }
      else if (Temp == 0x01) //修改模式
      {
        USART_Interrupt_flag = 0x01; //进入模式修改模式
        USART_Get_Num_Flag = 0;
      }
      else if (Temp == 0x02)
      {
        USART_Interrupt_flag = 0x02; //进入模式修改模式
        USART_Get_Num_Flag = 0;
      }
      break;
    case 0x00:
      USART_receive[USART_Get_Num_Flag] = Temp;
      USART_Get_Num_Flag++;
      if (USART_Get_Num_Flag > 4) //参数处理
      {
        PARAMETER_MODIFICATION(USART_receive);
        USART_Interrupt_flag = 0xff; //回到等待模式
      }
      break;
    case 0x01:
      USART_receive[USART_Get_Num_Flag] = Temp;
      USART_Get_Num_Flag++;
      if (USART_Get_Num_Flag > 4) //参数处理
      {
        MODE_MODIFICATION(USART_receive);
        USART_Interrupt_flag = 0xff; //回到等待模式
      }
      break;
    case 0x02:
      USART_receive[USART_Get_Num_Flag] = Temp;
      USART_Get_Num_Flag++;
      if (USART_Get_Num_Flag > 4) //参数处理
      {
        if (USART_receive[0] == 0x0a)
        {
          for (int j = 1; j < 5; j++)
          {
            if (USART_receive[j] != 0x0a)
              USART_Interrupt_flag = 0xff; //回到等待模式
          }
          if (USART_Interrupt_flag == 0x02)
          {
            On_Off_flag = 1;
            USART_Interrupt_flag = 0xff; //回到等待模式
          }
        }
        else if (USART_receive[0] == 0xb0)
        {
          for (int j = 1; j < 5; j++)
          {
            if (USART_receive[j] != 0xb0)
              USART_Interrupt_flag = 0xff; //回到等待模式
          }
          if (USART_Interrupt_flag == 0x02)
          {
            On_Off_flag = 0;
            USART_Interrupt_flag = 0xff; //回到等待模式
          }
        }
        else
          USART_Interrupt_flag = 0xff; //回到等待模式
      }
      break;

    default:
      USART_Interrupt_flag = 0xff; //回到等待模式
      break;
    }
  }
  return 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
