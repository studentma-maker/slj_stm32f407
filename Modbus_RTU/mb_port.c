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

/** Includes --------------------------------------------------------------------------------------------*/
#include "mb_port.h"
#include "tim.h"
#include "usart.h"

/**********************************************************************************************************
** Function name        :   mbs_port_uartInit
** Descriptions         :   MODBUS串口初始化接口
** parameters           :   huart: UART句柄
**                      :   usart: 类型
**                      :   baud:串口波特率
**						:	parity:奇偶校验位设置
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_uartInit(UART_HandleTypeDef *huart, USART_TypeDef *usart, uint32_t baud, uint8_t parity)
{
    /*串口部分初始化*/
    huart->Instance = usart;
    huart->Init.BaudRate = baud;
    
    if(parity == MB_PARITY_ODD)
    {
        huart->Init.WordLength = UART_WORDLENGTH_9B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_ODD;
    }
    else if(parity == MB_PARITY_EVEN)
    {
        huart->Init.WordLength = UART_WORDLENGTH_9B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_EVEN;
    }
    else
    {
        huart->Init.WordLength = UART_WORDLENGTH_8B;
        huart->Init.StopBits = UART_STOPBITS_1;
        huart->Init.Parity = UART_PARITY_NONE;
    }

    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK)
    {
        Error_Handler();
    }
}

/**********************************************************************************************************
** Function name        :   mbs_port_uartEnable
** Descriptions         :   串口TX\RX使能
** parameters           :   huart: UART句柄
**                      :   txen:0-关闭tx中断	1-打开tx中断
**						:	rxen:0-关闭rx中断	1-打开rx中断
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_uartEnable(UART_HandleTypeDef *huart, uint8_t txen, uint8_t rxen)
{
    if(txen)
    {
        __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
    }

    if(rxen)
    {
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    }
}

/**********************************************************************************************************
** Function name        :   mbs_port_putchar
** Descriptions         :   串口发送一个byte
** parameters           :   huart: UART句柄
**                      :   ch:要发送的byte
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_putchar(UART_HandleTypeDef *huart, uint8_t ch)
{
    huart->Instance->DR = ch;
}

/**********************************************************************************************************
** Function name        :   mbs_port_getchar
** Descriptions         :   串口读取一个byte
** parameters           :   huart: UART句柄
**                      :   ch:存放读取一个byte的指针
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_getchar(UART_HandleTypeDef *huart, uint8_t *ch)
{
    *ch = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
}

/**********************************************************************************************************
** Function name        :   mbs_port_timerInit
** Descriptions         :   定时器初始化
** parameters           :   htim: Timer句柄
**                      :   tim: 类型
**                      :   baud:串口波特率, 根据波特率生成3.5T的定时
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_timerInit(TIM_HandleTypeDef *htim, TIM_TypeDef *tim, uint32_t baud)
{
    /*定时器部分初始化*/
    htim->Instance = tim;
    htim->Init.Prescaler = 83; // 1us记一次数
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;

    if(baud > 19200) //波特率大于19200固定使用1800作为3.5T
    {
        htim->Init.Period = 1800;
    }
    else   //其他波特率的需要根据计算
    {
        /*	us=1s/(baud/11)*1000000*3.5
        *			=(11*1000000*3.5)/baud
        *			=38500000/baud
        */
        htim->Init.Period = (uint32_t)38500000 / baud;

    }

    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
}

/**********************************************************************************************************
** Function name        :   mbs_port_timerEnable
** Descriptions         :   定时器使能
** parameters           :   htim: Timer句柄
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_timerEnable(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);               //清除中断位
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);               //使能中断位
    __HAL_TIM_SET_COUNTER(htim, 0);                          //设置定时器计数为0
    __HAL_TIM_ENABLE(htim);                                 //使能定时器
}

/**********************************************************************************************************
** Function name        :   mbs_port_timerDisable
** Descriptions         :   定时器关闭
** parameters           :   htim: Timer句柄
** Returned value       :   无
***********************************************************************************************************/
void mbs_port_timerDisable(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
}

/**********************************************************************************************************
** Function name        :   USART_USB_IRQHandler
** Descriptions         :   串口中断服务函数
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void USART_USB_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART_USB_IRQ);

    if((__HAL_UART_GET_FLAG(&HUART_USB, UART_FLAG_RXNE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_USB, UART_FLAG_RXNE);
        mbs_uartRxIsr(&mbsUSB);
    }

    if((__HAL_UART_GET_FLAG(&HUART_USB, UART_FLAG_TXE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_USB, UART_FLAG_TXE);
        mbs_uartTxIsr(&mbsUSB);
    }
}

/**********************************************************************************************************
** Function name        :   USART_ESP_IRQHandler
** Descriptions         :   串口中断服务函数
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void USART_ESP_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART_ESP_IRQ);

    if((__HAL_UART_GET_FLAG(&HUART_ESP, UART_FLAG_RXNE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_ESP, UART_FLAG_RXNE);
        mbs_uartRxIsr(&mbsESP);
    }

    if((__HAL_UART_GET_FLAG(&HUART_ESP, UART_FLAG_TXE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_ESP, UART_FLAG_TXE);
        mbs_uartTxIsr(&mbsESP);
    }
}

/**********************************************************************************************************
** Function name        :   USART_STM_IRQHandler
** Descriptions         :   串口中断服务函数
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void USART_STM_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(USART_STM_IRQ);

    if((__HAL_UART_GET_FLAG(&HUART_STM, UART_FLAG_RXNE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_STM, UART_FLAG_RXNE);
        mbs_uartRxIsr(&mbsSTM);
    }

    if((__HAL_UART_GET_FLAG(&HUART_STM, UART_FLAG_TXE) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&HUART_STM, UART_FLAG_TXE);
        mbs_uartTxIsr(&mbsSTM);
    }
}

/**********************************************************************************************************
** Function name        :   TIM_USB_IRQHandler
** Descriptions         :   定时器中断服务函数
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void TIM_USB_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&HTIM_USB, TIM_IT_UPDATE);
    mbs_timer3T5Isr(&mbsUSB);
}


/**********************************************************************************************************
** Function name        :   TIM_ESP_IRQHandler
** Descriptions         :   定时器中断服务函数
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void TIM_ESP_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&HTIM_ESP, TIM_IT_UPDATE);
    mbs_timer3T5Isr(&mbsESP);
}

/**********************************************************************************************************
** Function name        :   TIM_STM_IRQHandler
** Descriptions         :   定时器中断服务函数
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void TIM_STM_IRQHandler(void)
{
    __HAL_TIM_CLEAR_IT(&HTIM_STM, TIM_IT_UPDATE);
    mbs_timer3T5Isr(&mbsSTM);
}
/********************************************** END OF FILE ***********************************************/
