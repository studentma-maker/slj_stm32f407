/*************************************** Copyright (c)******************************************************
** File name            :   mb_port.h
** Latest modified Date :   2025-06-20
** Latest Version       :   1.00
** Descriptions         :   modbus 驱动接口
**
**--------------------------------------------------------------------------------------------------------
** Created by           :   Ghailiu
** Created date         :   2025-06-20
** Version              :   1.00
** Descriptions         :   The original version
**
**--------------------------------------------------------------------------------------------------------
** Copyright            :
**
**********************************************************************************************************/
#ifndef _MB_PORTS_H
#define _MB_PORTS_H

/** Includes --------------------------------------------------------------------------------------------*/
#include "main.h"
#include "mb_slave.h"

/**********************************************************************************************************
** Descriptions :  接口定义
**********************************************************************************************************/
#define HTIM_USB                htim6
#define TIM_USB                 TIM6
#define TIM_USB_IRQHandler      TIM6_DAC_IRQHandler    // 定时器中断处理函数

#define HUART_USB               huart1
#define USART_USB               USART1
#define USART_USB_IRQ           USART1_IRQn
#define USART_USB_IRQHandler    USART1_IRQHandler  // 串口中断处理函数

#define HTIM_ESP                htim7
#define TIM_ESP                 TIM7
#define TIM_ESP_IRQHandler      TIM7_IRQHandler    // 定时器中断处理函数

#define HUART_ESP               huart2
#define USART_ESP               USART2
#define USART_ESP_IRQ           USART2_IRQn
#define USART_ESP_IRQHandler    USART2_IRQHandler  // 串口中断处理函数

#define HTIM_STM                htim12
#define TIM_STM                 TIM12
#define TIM_STM_IRQHandler      TIM8_BRK_TIM12_IRQHandler    // 定时器中断处理函数

#define HUART_STM               huart5
#define USART_STM               UART5
#define USART_STM_IRQ           UART5_IRQn
#define USART_STM_IRQHandler    UART5_IRQHandler  // 串口中断处理函数

/**********************************************************************************************************
** Descriptions :  校验位枚举
**********************************************************************************************************/
typedef enum
{
    MB_PARITY_NONE = 0X00,	// 无奇偶校验，两个停止位
    MB_PARITY_ODD, 			// 奇校验
    MB_PARITY_EVEN			// 偶校验
} mbParity;

/**********************************************************************************************************
** Function name        :   mbs_port_uartInit
** Descriptions         :   MODBUS串口初始化接口
** parameters           :   huart: UART句柄
**                      :   usart: 类型
**                      :   baud:串口波特率
**						:	parity:奇偶校验位设置
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_uartInit(UART_HandleTypeDef *huart, USART_TypeDef *usart, uint32_t baud, uint8_t parity);

/**********************************************************************************************************
** Function name        :   mbs_port_uartEnable
** Descriptions         :   串口TX\RX使能
** parameters           :   huart: UART句柄
**                      :   txen:0-关闭tx中断	1-打开tx中断
**						:	rxen:0-关闭rx中断	1-打开rx中断
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_uartEnable(UART_HandleTypeDef *huart, uint8_t txen, uint8_t rxen);

/**********************************************************************************************************
** Function name        :   mbs_port_putchar
** Descriptions         :   串口发送一个byte
** parameters           :   huart: UART句柄
**                      :   ch:要发送的byte
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_putchar(UART_HandleTypeDef *huart, uint8_t ch);

/**********************************************************************************************************
** Function name        :   mbs_port_getchar
** Descriptions         :   串口读取一个byte
** parameters           :   huart: UART句柄
**                      :   ch:存放读取一个byte的指针
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_getchar(UART_HandleTypeDef *huart, uint8_t *ch);

/**********************************************************************************************************
** Function name        :   mbs_port_timerInit
** Descriptions         :   定时器初始化
** parameters           :   htim: Timer句柄
**                      :   tim: 类型
**                      :   baud:串口波特率, 根据波特率生成3.5T的定时
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_timerInit(TIM_HandleTypeDef *htim, TIM_TypeDef *tim, uint32_t baud);

/**********************************************************************************************************
** Function name        :   mbs_port_timerEnable
** Descriptions         :   定时器使能
** parameters           :   htim: Timer句柄
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_timerEnable(TIM_HandleTypeDef *htim);

/**********************************************************************************************************
** Function name        :   mbs_port_timerDisable
** Descriptions         :   定时器关闭
** parameters           :   htim: Timer句柄
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_timerDisable(TIM_HandleTypeDef *htim);

/**********************************************************************************************************
** Function name        :   mbs_port_uartInit
** Descriptions         :   MODBUS串口初始化接口
** parameters           :   baud:串口波特率
**						:	parity:奇偶校验位设置
** Returned value       :   无
***********************************************************************************************************/
void mbs_ext_port_uartInit(uint32_t baud, uint8_t parity);

/**********************************************************************************************************
** Function name        :   mbs_port_uartEnable
** Descriptions         :   串口TX\RX使能
** parameters           :   txen:0-关闭tx中断	1-打开tx中断
**						:	rxen:0-关闭rx中断	1-打开rx中断
** Returned value       :   无
***********************************************************************************************************/
void mbs_ext_port_uartEnable(uint8_t txen, uint8_t rxen);

/**********************************************************************************************************
** Function name        :   mbs_port_putchar
** Descriptions         :   串口发送一个byte
** parameters           :   ch:要发送的byte
** Returned value       :   无
***********************************************************************************************************/
void mbs_ext_port_putchar(uint8_t ch);

/**********************************************************************************************************
** Function name        :   mbs_port_getchar
** Descriptions         :   串口读取一个byte
** parameters           :   ch:存放读取一个byte的指针
** Returned value       :   无
***********************************************************************************************************/
void mbs_ext_port_getchar(uint8_t *ch);

/**********************************************************************************************************
** Function name        :   mbs_port_timerInit
** Descriptions         :   定时器初始化
** parameters           :   baud:串口波特率, 根据波特率生成3.5T的定时
** Returned value       :   无
***********************************************************************************************************/
void mbs_ext_port_timerInit(uint32_t baud);

/**********************************************************************************************************
** Function name        :   mbs_port_timerEnable
** Descriptions         :   定时器使能
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_ext_port_timerEnable(void);

/**********************************************************************************************************
** Function name        :   mbs_port_timerDisable
** Descriptions         :   定时器关闭
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_ext_port_timerDisable(void);

/**********************************************************************************************************
** Function name        :   mbs_port_uartInit
** Descriptions         :   MODBUS串口初始化接口
** parameters           :   baud:串口波特率
**						:	parity:奇偶校验位设置
** Returned value       :   无
***********************************************************************************************************/
void mbs_hmi_port_uartInit(uint32_t baud, uint8_t parity);

/**********************************************************************************************************
** Function name        :   mbs_port_uartEnable
** Descriptions         :   串口TX\RX使能
** parameters           :   txen:0-关闭tx中断	1-打开tx中断
**						:	rxen:0-关闭rx中断	1-打开rx中断
** Returned value       :   无
***********************************************************************************************************/
void mbs_hmi_port_uartEnable(uint8_t txen, uint8_t rxen);

/**********************************************************************************************************
** Function name        :   mbs_port_putchar
** Descriptions         :   串口发送一个byte
** parameters           :   ch:要发送的byte
** Returned value       :   无
***********************************************************************************************************/
void mbs_hmi_port_putchar(uint8_t ch);

/**********************************************************************************************************
** Function name        :   mbs_port_getchar
** Descriptions         :   串口读取一个byte
** parameters           :   ch:存放读取一个byte的指针
** Returned value       :   无
***********************************************************************************************************/
void mbs_hmi_port_getchar(uint8_t *ch);

/**********************************************************************************************************
** Function name        :   mbs_port_timerInit
** Descriptions         :   定时器初始化
** parameters           :   baud:串口波特率, 根据波特率生成3.5T的定时
** Returned value       :   无
***********************************************************************************************************/
void mbs_hmi_port_timerInit(uint32_t baud);

/**********************************************************************************************************
** Function name        :   mbs_port_timerEnable
** Descriptions         :   定时器使能
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_hmi_port_timerEnable(void);

/**********************************************************************************************************
** Function name        :   mbs_port_timerDisable
** Descriptions         :   定时器关闭
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_hmi_port_timerDisable(void);

#endif // _MB_PORTS_H

/********************************************** END OF FILE ***********************************************/
