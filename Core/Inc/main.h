/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN5_Pin GPIO_PIN_2
#define IN5_GPIO_Port GPIOE
#define IN4_Pin GPIO_PIN_3
#define IN4_GPIO_Port GPIOE
#define IN3_Pin GPIO_PIN_4
#define IN3_GPIO_Port GPIOE
#define IN2_Pin GPIO_PIN_5
#define IN2_GPIO_Port GPIOE
#define IN1_Pin GPIO_PIN_6
#define IN1_GPIO_Port GPIOE
#define SMD_DR_5_Pin GPIO_PIN_4
#define SMD_DR_5_GPIO_Port GPIOC
#define SMD_DR_6_Pin GPIO_PIN_5
#define SMD_DR_6_GPIO_Port GPIOC
#define SMD_DR_1_Pin GPIO_PIN_2
#define SMD_DR_1_GPIO_Port GPIOB
#define SMD_DR_2_Pin GPIO_PIN_11
#define SMD_DR_2_GPIO_Port GPIOF
#define SMD_EN_1_Pin GPIO_PIN_12
#define SMD_EN_1_GPIO_Port GPIOF
#define SMD_EN_2_Pin GPIO_PIN_13
#define SMD_EN_2_GPIO_Port GPIOF
#define SMD_AM_2_Pin GPIO_PIN_14
#define SMD_AM_2_GPIO_Port GPIOF
#define SMD_AM_1_Pin GPIO_PIN_15
#define SMD_AM_1_GPIO_Port GPIOF
#define SMD_DR_3_Pin GPIO_PIN_11
#define SMD_DR_3_GPIO_Port GPIOE
#define SMD_DR_4_Pin GPIO_PIN_12
#define SMD_DR_4_GPIO_Port GPIOE
#define SMD_EN_3_Pin GPIO_PIN_13
#define SMD_EN_3_GPIO_Port GPIOE
#define SMD_EN_4_Pin GPIO_PIN_14
#define SMD_EN_4_GPIO_Port GPIOE
#define SMD_AM_4_Pin GPIO_PIN_15
#define SMD_AM_4_GPIO_Port GPIOE
#define SMD_AM_3_Pin GPIO_PIN_10
#define SMD_AM_3_GPIO_Port GPIOB
#define SMD_EN_5_Pin GPIO_PIN_11
#define SMD_EN_5_GPIO_Port GPIOB
#define SMD_EN_6_Pin GPIO_PIN_12
#define SMD_EN_6_GPIO_Port GPIOB
#define SMD_AM_6_Pin GPIO_PIN_13
#define SMD_AM_6_GPIO_Port GPIOB
#define SMD_AM_5_Pin GPIO_PIN_14
#define SMD_AM_5_GPIO_Port GPIOB
#define SMD_DR_7_Pin GPIO_PIN_15
#define SMD_DR_7_GPIO_Port GPIOB
#define SMD_DR_8_Pin GPIO_PIN_8
#define SMD_DR_8_GPIO_Port GPIOD
#define SMD_EN_7_Pin GPIO_PIN_9
#define SMD_EN_7_GPIO_Port GPIOD
#define SMD_EN_8_Pin GPIO_PIN_10
#define SMD_EN_8_GPIO_Port GPIOD
#define SMD_AM_8_Pin GPIO_PIN_11
#define SMD_AM_8_GPIO_Port GPIOD
#define SMD_AM_7_Pin GPIO_PIN_12
#define SMD_AM_7_GPIO_Port GPIOD
#define OUT16_Pin GPIO_PIN_14
#define OUT16_GPIO_Port GPIOD
#define OUT15_Pin GPIO_PIN_15
#define OUT15_GPIO_Port GPIOD
#define OUT14_Pin GPIO_PIN_2
#define OUT14_GPIO_Port GPIOG
#define OUT13_Pin GPIO_PIN_3
#define OUT13_GPIO_Port GPIOG
#define OUT12_Pin GPIO_PIN_4
#define OUT12_GPIO_Port GPIOG
#define OUT11_Pin GPIO_PIN_5
#define OUT11_GPIO_Port GPIOG
#define OUT10_Pin GPIO_PIN_6
#define OUT10_GPIO_Port GPIOG
#define OUT9_Pin GPIO_PIN_7
#define OUT9_GPIO_Port GPIOG
#define OUT8_Pin GPIO_PIN_8
#define OUT8_GPIO_Port GPIOG
#define OUT7_Pin GPIO_PIN_6
#define OUT7_GPIO_Port GPIOC
#define OUT6_Pin GPIO_PIN_7
#define OUT6_GPIO_Port GPIOC
#define OUT5_Pin GPIO_PIN_8
#define OUT5_GPIO_Port GPIOC
#define OUT4_Pin GPIO_PIN_9
#define OUT4_GPIO_Port GPIOC
#define OUT3_Pin GPIO_PIN_8
#define OUT3_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_11
#define OUT2_GPIO_Port GPIOA
#define OUT1_Pin GPIO_PIN_12
#define OUT1_GPIO_Port GPIOA
#define IN20_Pin GPIO_PIN_15
#define IN20_GPIO_Port GPIOA
#define IN19_Pin GPIO_PIN_10
#define IN19_GPIO_Port GPIOC
#define IN18_Pin GPIO_PIN_11
#define IN18_GPIO_Port GPIOC
#define IN17_Pin GPIO_PIN_0
#define IN17_GPIO_Port GPIOD
#define IN16_Pin GPIO_PIN_1
#define IN16_GPIO_Port GPIOD
#define IN15_Pin GPIO_PIN_3
#define IN15_GPIO_Port GPIOD
#define IN14_Pin GPIO_PIN_4
#define IN14_GPIO_Port GPIOD
#define IN13_Pin GPIO_PIN_7
#define IN13_GPIO_Port GPIOD
#define IN12_Pin GPIO_PIN_9
#define IN12_GPIO_Port GPIOG
#define IN11_Pin GPIO_PIN_10
#define IN11_GPIO_Port GPIOG
#define IN10_Pin GPIO_PIN_11
#define IN10_GPIO_Port GPIOG
#define IN9_Pin GPIO_PIN_12
#define IN9_GPIO_Port GPIOG
#define IN8_Pin GPIO_PIN_13
#define IN8_GPIO_Port GPIOG
#define IN7_Pin GPIO_PIN_14
#define IN7_GPIO_Port GPIOG
#define IN6_Pin GPIO_PIN_1
#define IN6_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
// 普通输入读取：IN_READ(索引) → 返回 0（低）/1（高）
// 索引：0~19 → 对应IN1~IN20（0起始）
#define IN_READ(index) ( \
    (index <= 19) ? \
    (HAL_GPIO_ReadPin( \
        /* IN1~IN20 端口映射表 */ \
        ((GPIO_TypeDef*[]) { \
            IN1_GPIO_Port, IN2_GPIO_Port, IN3_GPIO_Port, IN4_GPIO_Port, IN5_GPIO_Port, \
            IN6_GPIO_Port, IN7_GPIO_Port, IN8_GPIO_Port, IN9_GPIO_Port, IN10_GPIO_Port, \
            IN11_GPIO_Port, IN12_GPIO_Port, IN13_GPIO_Port, IN14_GPIO_Port, IN15_GPIO_Port, \
            IN16_GPIO_Port, IN17_GPIO_Port, IN18_GPIO_Port, IN19_GPIO_Port, IN20_GPIO_Port \
        })[index], \
        /* IN1~IN20 引脚映射表 */ \
        ((uint16_t[]) { \
            IN1_Pin, IN2_Pin, IN3_Pin, IN4_Pin, IN5_Pin, \
            IN6_Pin, IN7_Pin, IN8_Pin, IN9_Pin, IN10_Pin, \
            IN11_Pin, IN12_Pin, IN13_Pin, IN14_Pin, IN15_Pin, \
            IN16_Pin, IN17_Pin, IN18_Pin, IN19_Pin, IN20_Pin \
        })[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

// 报警输入读取：ALARM_READ(索引) → 返回 0（低）/1（高）
// 索引：0~7 → 对应SMD_AM_1~SMD_AM_8（0起始）
#define SMD_AM_READ(index) ( \
    (index <= 7) ? \
    (HAL_GPIO_ReadPin( \
        /* SMD_AM_1~8 端口映射表 */ \
        ((GPIO_TypeDef*[]) { \
            SMD_AM_1_GPIO_Port, SMD_AM_2_GPIO_Port, SMD_AM_3_GPIO_Port, SMD_AM_4_GPIO_Port, \
            SMD_AM_5_GPIO_Port, SMD_AM_6_GPIO_Port, SMD_AM_7_GPIO_Port, SMD_AM_8_GPIO_Port \
        })[index], \
        /* SMD_AM_1~8 引脚映射表 */ \
        ((uint16_t[]) { \
            SMD_AM_1_Pin, SMD_AM_2_Pin, SMD_AM_3_Pin, SMD_AM_4_Pin, \
            SMD_AM_5_Pin, SMD_AM_6_Pin, SMD_AM_7_Pin, SMD_AM_8_Pin \
        })[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

// 核心宏：OUT(索引, 电平)
// 索引：0~15 → 对应OUT1~OUT16（0起始）
// 电平：0=低电平，1=高电平，2=翻转电平
#define OUT(index, level) do{ \
    /* 引脚映射表：索引0→OUT1，索引1→OUT2...索引15→OUT16 */ \
    GPIO_TypeDef* OUT_PORT[] = { \
        OUT1_GPIO_Port, OUT2_GPIO_Port, OUT3_GPIO_Port, OUT4_GPIO_Port, \
        OUT5_GPIO_Port, OUT6_GPIO_Port, OUT7_GPIO_Port, OUT8_GPIO_Port, \
        OUT9_GPIO_Port, OUT10_GPIO_Port, OUT11_GPIO_Port, OUT12_GPIO_Port, \
        OUT13_GPIO_Port, OUT14_GPIO_Port, OUT15_GPIO_Port, OUT16_GPIO_Port \
    }; \
    uint16_t OUT_PIN[] = { \
        OUT1_Pin, OUT2_Pin, OUT3_Pin, OUT4_Pin, \
        OUT5_Pin, OUT6_Pin, OUT7_Pin, OUT8_Pin, \
        OUT9_Pin, OUT10_Pin, OUT11_Pin, OUT12_Pin, \
        OUT13_Pin, OUT14_Pin, OUT15_Pin, OUT16_Pin \
    }; \
    /* 防越界：仅处理0~15的索引 */ \
    if(index <= 15) { \
        if(level == 0) { \
            HAL_GPIO_WritePin(OUT_PORT[index], OUT_PIN[index], GPIO_PIN_RESET); \
        } else { \
            HAL_GPIO_WritePin(OUT_PORT[index], OUT_PIN[index], GPIO_PIN_SET); \
        } \
    } \
}while(0)

// 输出读取：OUT_READ(索引) → 返回 0（低）/1（高）
// 索引：0~15 → 对应OUT1~OUT16（0起始）
#define OUT_READ(index) ( \
    (index <= 15) ? \
    (HAL_GPIO_ReadPin( \
        ((GPIO_TypeDef*[]) {OUT1_GPIO_Port, OUT2_GPIO_Port, OUT3_GPIO_Port, OUT4_GPIO_Port, \
         OUT5_GPIO_Port, OUT6_GPIO_Port, OUT7_GPIO_Port, OUT8_GPIO_Port, \
         OUT9_GPIO_Port, OUT10_GPIO_Port, OUT11_GPIO_Port, OUT12_GPIO_Port, \
         OUT13_GPIO_Port, OUT14_GPIO_Port, OUT15_GPIO_Port, OUT16_GPIO_Port})[index], \
        ((uint16_t[]) {OUT1_Pin, OUT2_Pin, OUT3_Pin, OUT4_Pin, \
         OUT5_Pin, OUT6_Pin, OUT7_Pin, OUT8_Pin, \
         OUT9_Pin, OUT10_Pin, OUT11_Pin, OUT12_Pin, \
         OUT13_Pin, OUT14_Pin, OUT15_Pin, OUT16_Pin})[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

// SMD_DR支持双参数：SMD_DR(索引, 电平)
// 索引：0~7 → 对应SMD_DR_1~SMD_DR_8，电平：0=低/1=高/2=翻转
#define SMD_DR(index, level) do{ \
    GPIO_TypeDef* SMD_DR_PORT[] = { \
        SMD_DR_1_GPIO_Port, SMD_DR_2_GPIO_Port, SMD_DR_3_GPIO_Port, SMD_DR_4_GPIO_Port, \
        SMD_DR_5_GPIO_Port, SMD_DR_6_GPIO_Port, SMD_DR_7_GPIO_Port, SMD_DR_8_GPIO_Port \
    }; \
    uint16_t SMD_DR_PIN[] = { \
        SMD_DR_1_Pin, SMD_DR_2_Pin, SMD_DR_3_Pin, SMD_DR_4_Pin, \
        SMD_DR_5_Pin, SMD_DR_6_Pin, SMD_DR_7_Pin, SMD_DR_8_Pin \
    }; \
    if(index <= 7) { \
        if(level == 0) { \
            HAL_GPIO_WritePin(SMD_DR_PORT[index], SMD_DR_PIN[index], GPIO_PIN_RESET); \
        } else { \
            HAL_GPIO_WritePin(SMD_DR_PORT[index], SMD_DR_PIN[index], GPIO_PIN_SET); \
        } \
    } \
}while(0)

#define SMD_DR_READ(index) ( \
    (index <= 7) ? \
    (HAL_GPIO_ReadPin( \
        ((GPIO_TypeDef*[]) {SMD_DR_1_GPIO_Port, SMD_DR_2_GPIO_Port, SMD_DR_3_GPIO_Port, SMD_DR_4_GPIO_Port, \
         SMD_DR_5_GPIO_Port, SMD_DR_6_GPIO_Port, SMD_DR_7_GPIO_Port, SMD_DR_8_GPIO_Port})[index], \
        ((uint16_t[]) {SMD_DR_1_Pin, SMD_DR_2_Pin, SMD_DR_3_Pin, SMD_DR_4_Pin, \
         SMD_DR_5_Pin, SMD_DR_6_Pin, SMD_DR_7_Pin, SMD_DR_8_Pin})[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

// SMD_EN支持双参数：SMD_EN(索引, 电平)
#define SMD_EN(index, level) do{ \
    GPIO_TypeDef* SMD_EN_PORT[] = { \
        SMD_EN_1_GPIO_Port, SMD_EN_2_GPIO_Port, SMD_EN_3_GPIO_Port, SMD_EN_4_GPIO_Port, \
        SMD_EN_5_GPIO_Port, SMD_EN_6_GPIO_Port, SMD_EN_7_GPIO_Port, SMD_EN_8_GPIO_Port \
    }; \
    uint16_t SMD_EN_PIN[] = { \
        SMD_EN_1_Pin, SMD_EN_2_Pin, SMD_EN_3_Pin, SMD_EN_4_Pin, \
        SMD_EN_5_Pin, SMD_EN_6_Pin, SMD_EN_7_Pin, SMD_EN_8_Pin \
    }; \
    if(index <= 7) { \
        if(level == 0) { \
            HAL_GPIO_WritePin(SMD_EN_PORT[index], SMD_EN_PIN[index], GPIO_PIN_RESET); \
        } else { \
            HAL_GPIO_WritePin(SMD_EN_PORT[index], SMD_EN_PIN[index], GPIO_PIN_SET); \
        } \
    } \
}while(0)

#define SMD_EN_READ(index) ( \
    (index <= 7) ? \
    (HAL_GPIO_ReadPin( \
        ((GPIO_TypeDef*[]) {SMD_EN_1_GPIO_Port, SMD_EN_2_GPIO_Port, SMD_EN_3_GPIO_Port, SMD_EN_4_GPIO_Port, \
         SMD_EN_5_GPIO_Port, SMD_EN_6_GPIO_Port, SMD_EN_7_GPIO_Port, SMD_EN_8_GPIO_Port})[index], \
        ((uint16_t[]) {SMD_EN_1_Pin, SMD_EN_2_Pin, SMD_EN_3_Pin, SMD_EN_4_Pin, \
         SMD_EN_5_Pin, SMD_EN_6_Pin, SMD_EN_7_Pin, SMD_EN_8_Pin})[index] \
    ) == GPIO_PIN_SET ? 1 : 0) : 0 \
)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
