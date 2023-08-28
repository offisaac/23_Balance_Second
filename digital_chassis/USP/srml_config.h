/**
  ******************************************************************************
  * @file    srml_config_template.h
  * @brief   SRML configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to srml_config.h.
  * @version 0.0.1
  ******************************************************************************
  * MOST IMPORTANTLY, this library is not open source for developers from other
  * schools' robot team currently. Plz make sure your code is under your supervision.
  *
  * Thank for @mannychen @HuanpengLu and other pioneers who have put forward such
  * architechure, and the endeavors of all developers.
  *
  * By downloading, copying, installing or using the software you agree to this license.
  * If you do not agree to this license, do not download, install,
  * copy or use the software.
  *
  *                          License Agreement
  *                For SCUT RobotLab Middleware Layer Library
  *
  * Copyright (c) 2019 - ~, SCUT RobotLab Development Team, all rights reserved.
  */

#ifndef __SRML_CONFIG_H__
#define __SRML_CONFIG_H__

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the SRML.
  *        Change the value to 1 to use it.
  */

/* Drivers ----------------------------------------------------*/
#define USE_SRML_CAN                      1
#define USE_SRML_TIMER                    1
#define USE_SRML_UART                     1
#define USE_SRML_FLASH                    0
#define USE_SRML_I2C                      0
#define USE_SRML_SPI                      0

#define USE_SRML_MOTOR_DJI                1
#define USE_SRML_MF9025_V2                1
#define USE_SRML_HT04                     1
#define USE_SRML_DR16                     0
#define USE_SRML_LPMS_BE2                 1
#define USE_SRML_MPU6050                  0
#define USE_SRML_REFEREE                  1

/******************************** 几乎不用 ********************************/
#define USE_SRML_BMX055                   0
#define USE_SRML_FATFS                    0
#define USE_SRML_W25Qx                    0
#define USE_SRML_MOTOR_AK80               0
#define USE_SRML_VSEC                     0
/******************************** 几乎不用 ********************************/

/* Middlewares -----------------------------------------------*/
#define USE_SRML_ABS_LIB                  1
#define USE_SRML_FILTER                   1
#define USE_SRML_DIFF_CALCULATER          1
#define USE_SRML_PID                      1
#define USE_SRML_DIGITAL_POWER            1
#define USE_SRML_CHASSIS                  0

/******************************** 几乎不用 ********************************/
#define USE_SRML_MOTOR_CTRL               0
#define USE_SRML_POW_CTRL                 0
#define USE_SRML_SERIAL_LINE_IP           0
#define USE_SRML_ASUWAVE                  0
#define USE_SRML_LIST                     0
#define USE_SRML_MYASSERT                 0
#define USE_SRML_SYSANALYSIS              0
#define USE_SRML_SYSLOG                   0
/******************************** 几乎不用 ********************************/

#endif /* __SRML_CONFIG_H__ */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
