/*************************************** Copyright (c)******************************************************
** File name            :   mb_slave.h
** Latest modified Date :   2025-06-20
** Latest Version       :   1.00
** Descriptions         :   modbus 从机程序
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
#ifndef _MB_SLAVE_H
#define _MB_SLAVE_H

/** Includes --------------------------------------------------------------------------------------------*/
#include "main.h"
#include "mb_port.h"

/**********************************************************************************************************
** Descriptions :  参数定义
**********************************************************************************************************/
#define MBS_RTU_MIN_SIZE	4
#define MBS_RTU_MAX_SIZE	510 // 最大不超过255
#define MBS_ERR_MAX_TIMES	3
#define MBS_REC_TIMEOUT		100 // 单位3.5T

#define REG_HOLDING_NREGS   200 // 保持寄存器数量
#define REG_COILS_SIZE      16  // 线圈数量

/**********************************************************************************************************
** Descriptions :  从机结构体定义
**********************************************************************************************************/
typedef struct
{
    uint8_t state;						        // modbus 状态
    uint8_t txLen;     					        // 需要发送的帧长度
    uint8_t txCounter;					        // 已发送bytes计数
    uint8_t txBuf[MBS_RTU_MAX_SIZE];	        // 发送缓冲区
    uint8_t rxCounter;					        // 接收计数
    uint8_t rxBuf[MBS_RTU_MAX_SIZE];	        // 接收缓冲区
    uint8_t rspCode;                            // 应答码
    uint8_t regCoilsBuf[REG_COILS_SIZE];        // 线圈状态
    uint16_t regHoldingBuf[REG_HOLDING_NREGS];  // 保持寄存器数据
    
    uint8_t slaveAddr;                          // 地址码
    uint8_t parity;                             // 校验位
    uint32_t baudRate;                          // 波特兰
    
    UART_HandleTypeDef huart;
    USART_TypeDef *USART;
    
    TIM_HandleTypeDef htim;
    TIM_TypeDef *TIM;
    
    void (*putchar)(UART_HandleTypeDef *, uint8_t);             // 发送数据函数
    void (*getchar)(UART_HandleTypeDef *, uint8_t *);           // 接收数据函数
    void (*uartEnable)(UART_HandleTypeDef *, uint8_t, uint8_t); // 发送接收使能函数
    void (*timerEnable)(TIM_HandleTypeDef *);                   // 定时器开启
    void (*timerDisable)(TIM_HandleTypeDef *);                  // 定时器关闭
} mbs;

/**********************************************************************************************************
** Descriptions :  异常枚举定义
**********************************************************************************************************/
typedef enum
{
    RSP_OK = 0,         // 成功
    RSP_ERR_CMD,        // 不支持的功能码
    RSP_ERR_REG_ADDR,   // 寄存器地址错误
    RSP_ERR_VALUE,      // 数据值域错误
    RSP_ERR_WRITE       // 写入失败
    
} mb_slave_rsp;

/**********************************************************************************************************
** Descriptions :  状态枚举定义
**********************************************************************************************************/
typedef enum
{
    MBS_STATE_IDLE = 0X00,
    MBS_STATE_TX,           // 发送状态
    MBS_STATE_TX_END,       // 发送结束
    MBS_STATE_RX,           // 接收状态
    MBS_STATE_RX_CHECK,     // 接收检验
    MBS_STATE_EXEC,         // 接收回调
    MBS_STATE_REC_ERR,		// 接收错误
    MBS_STATE_TIMES_ERR,	// 超时错误
    
} mb_slave_state;

/**********************************************************************************************************
** Descriptions :  外部变量定义
**********************************************************************************************************/
extern mbs mbsUSB, mbsESP, mbsSTM;

/**********************************************************************************************************
** Function name        :   mbs_ext_init
** Descriptions         :   MODBUS从机初始化
** parameters           :   _mbs: 通信参数
** Returned value       :   无
***********************************************************************************************************/
void mbs_init(mbs *_mbs);

/**********************************************************************************************************
** Function name        :   mbs_poll
** Descriptions         :   MODBUS状态轮训
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_poll(mbs *_mbs);

/**********************************************************************************************************
** Function name        :   mbs_timer3T5Isr
** Descriptions         :   modbus定时器中断处理
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_timer3T5Isr(mbs *_mbs);

/**********************************************************************************************************
** Function name        :   mbs_uartRxIsr
** Descriptions         :   modbus串口接收中断处理
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_uartRxIsr(mbs *_mbs);

/**********************************************************************************************************
** Function name        :   mbs_uartRxIsr
** Descriptions         :   modbus串口发送中断处理
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_uartTxIsr(mbs *_mbs);

#endif

/********************************************** END OF FILE ***********************************************/
