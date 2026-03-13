/*************************************** Copyright (c)******************************************************
** File name            :   mb_slave.c
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

/** Includes --------------------------------------------------------------------------------------------*/
#include "mb_slave.h"
#include "mb_hook.h"
#include "mb_crc.h"
#include "string.h"
#include "usart.h"
#include "tim.h"

/** Private variables -----------------------------------------------------------------------------------*/
// 从机通信参数
mbs mbsUSB, mbsESP, mbsSTM;

/**********************************************************************************************************
** Function name        :   mbs_ext_init
** Descriptions         :   MODBUS从机初始化
** parameters           :   _mbs: 通信参数
** Returned value       :   无
***********************************************************************************************************/
void mbs_init(mbs *_mbs)
{
    if(_mbs == &mbsUSB)
    {
        _mbs->huart = HUART_USB;
        _mbs->USART = USART_USB;
        _mbs->htim = HTIM_USB;
        _mbs->TIM = TIM_USB;
    }
    else if(_mbs == &mbsESP)
    {
        _mbs->huart = HUART_ESP;
        _mbs->USART = USART_ESP;
        _mbs->htim = HTIM_ESP;
        _mbs->TIM = TIM_ESP;
    }
    else if(_mbs == &mbsSTM)
    {
        _mbs->huart = HUART_STM;
        _mbs->USART = USART_STM;
        _mbs->htim = HTIM_STM;
        _mbs->TIM = TIM_STM;
    }

    mbs_port_uartInit(&_mbs->huart, _mbs->USART, _mbs->baudRate, _mbs->parity);
    mbs_port_timerInit(&_mbs->htim, _mbs->TIM, _mbs->baudRate);
    
    _mbs->putchar = mbs_port_putchar;
    _mbs->getchar = mbs_port_getchar;
    _mbs->uartEnable = mbs_port_uartEnable;
    _mbs->timerEnable = mbs_port_timerEnable;
    _mbs->timerDisable = mbs_port_timerDisable;
    
    _mbs->state = MBS_STATE_RX;
    mbs_port_uartEnable(&_mbs->huart, 0, 1);
}

/**********************************************************************************************************
** Function name        :   mbs_send
** Descriptions         :   MODBUS主机给从机发送一条命令
** parameters           :   data:要发送的数据
**						:	len:发送的数据长度
** Returned value       :   -1:发送失败	0:发送成功
***********************************************************************************************************/
void mbs_send(mbs *_mbs)
{
    uint16_t crc;

    _mbs->txCounter = 0;
    _mbs->rxCounter = 0;
    crc = mb_crc16(_mbs->txBuf, _mbs->txLen);
    _mbs->txBuf[_mbs->txLen++] = (uint8_t)(crc & 0xff);
    _mbs->txBuf[_mbs->txLen++] = (uint8_t)(crc >> 8);

    _mbs->state = MBS_STATE_TX;
    _mbs->uartEnable(&_mbs->huart, 1, 0); //enable tx, disable rx
}

/**********************************************************************************************************
** Function name        :   mods_01h
** Descriptions         :   读取线圈状态
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
static void mods_01h(mbs *_mbs)
{
    /*
     举例：
        主机发送:
        11 从机地址
        01 功能码
        00 寄存器起始地址高字节
        13 寄存器起始地址低字节
        00 寄存器数量高字节
        25 寄存器数量低字节
        0E CRC校验高字节
        84 CRC校验低字节

        从机应答: 	1代表ON，0代表OFF。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
        11 从机地址
        01 功能码
        05 返回字节数
        CD 数据1(线圈0013H-线圈001AH)
        6B 数据2(线圈001BH-线圈0022H)
        B2 数据3(线圈0023H-线圈002AH)
        0E 数据4(线圈0032H-线圈002BH)
        1B 数据5(线圈0037H-线圈0033H)
        45 CRC校验高字节
        E6 CRC校验低字节

        例子:
        01 01 10 01 00 03   29 0B	--- 查询D01开始的3个继电器状态
        01 01 10 03 00 01   09 0A   --- 查询D03继电器的状态
    */
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint16_t m;
    uint8_t coilsBuffer[REG_COILS_SIZE / 8];

    memset(coilsBuffer, 0, REG_COILS_SIZE / 8);

    /* 没有外部继电器，直接应答错误 */
    if (_mbs->rxCounter != 8)
    {
        _mbs->rspCode = RSP_ERR_VALUE; // 数据值域错误/
        return;
    }

    reg = ((uint16_t)_mbs->rxBuf[2] << 8) | _mbs->rxBuf[3];  // 寄存器号
    num = ((uint16_t)_mbs->rxBuf[4] << 8) | _mbs->rxBuf[5];  // 寄存器个数

    m = (num + 7) / 8;
    
    /* 如果改变的数据在寄存器范围内,则更新线圈状态 */
    if ((num > 0) && (reg + num <= REG_COILS_SIZE))
    {
        mbs_hook_updata_coils(_mbs);  // 更新线圈状态

        for (i = 0; i < num; i++)
        {
            if ((_mbs->regCoilsBuf[(reg + i) / 8] >> ((reg + i) % 8)) & 0x01)  /* 读状态寄存器的每一位 */
            {
                coilsBuffer[i / 8] |= (1 << (i % 8));
            }
        }
    }
    else
    {
        _mbs->rspCode = RSP_ERR_REG_ADDR;  // 寄存器地址错误
    }

    if (_mbs->rspCode == RSP_OK)  // 正确应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1];
        _mbs->txBuf[_mbs->txLen++] = m;  // 返回字节数

        for (i = 0; i < m; i++)
        {
            _mbs->txBuf[_mbs->txLen++] = coilsBuffer[i];  // 线圈状态
        }
    }
    else // 错误应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1] | 0x80;
        _mbs->txBuf[_mbs->txLen++] = _mbs->rspCode;
    }
    
    mbs_send(_mbs); // 发送应答
}

/**********************************************************************************************************
** Function name        :   mods_03h
** Descriptions         :   读取保持寄存器
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mods_03h(mbs *_mbs)
{
    /*
        主机发送:
        11 从机地址
        03 功能码
        00 寄存器地址高字节
        6B 寄存器地址低字节
        00 寄存器数量高字节
        03 寄存器数量低字节
        76 CRC高字节
        87 CRC低字节

        从机应答: 	保持寄存器的长度为2个字节。对于单个保持寄存器而言，寄存器高字节数据先被传输，
                低字节数据后被传输。保持寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
        11 从机地址
        03 功能码
        06 字节数
        00 数据1高字节(006BH)
        6B 数据1低字节(006BH)
        00 数据2高字节(006CH)
        13 数据2 低字节(006CH)
        00 数据3高字节(006DH)
        00 数据3低字节(006DH)
        38 CRC高字节
        B9 CRC低字节
    */
    uint16_t reg;
    uint16_t num;
    uint16_t i;

    if (_mbs->rxCounter != 8)             // 03H命令必须是8个字节
    {
        _mbs->rspCode = RSP_ERR_VALUE;    // 数值域错误
    }

    reg = ((uint16_t)_mbs->rxBuf[2] << 8) | _mbs->rxBuf[3];  // 寄存器号
    num = ((uint16_t)_mbs->rxBuf[4] << 8) | _mbs->rxBuf[5];  // 寄存器个数

    /* 如果改变的数据在寄存器范围内,则更新数据 */
    if ((num > 0) && (reg + num <= REG_HOLDING_NREGS))
    {
        mbs_hook_updata_holding(_mbs);  // 更新寄存器数据
    }
    else
    {
        _mbs->rspCode = RSP_ERR_REG_ADDR;  // 寄存器地址错误
    }

    if (_mbs->rspCode == RSP_OK)  // 正确应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1];
        _mbs->txBuf[_mbs->txLen++] = num * 2;  // 返回字节数

        for (i = 0; i < num; i++)
        {
            _mbs->txBuf[_mbs->txLen++] = _mbs->regHoldingBuf[reg] >> 8;
            _mbs->txBuf[_mbs->txLen++] = _mbs->regHoldingBuf[reg] & 0xFF;
            reg++;
        }
    }
    else // 错误应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1] | 0x80;
        _mbs->txBuf[_mbs->txLen++] = _mbs->rspCode;
    }
    
    mbs_send(_mbs); // 发送应答
}

/**********************************************************************************************************
** Function name        :   mods_05h
** Descriptions         :   强制单线圈
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
static void mods_05h(mbs *_mbs)
{
    /*
        主机发送: 写单个线圈寄存器。FF00H值请求线圈处于ON状态，0000H值请求线圈处于OFF状态
        。05H指令设置单个线圈的状态，15H指令可以设置多个线圈的状态。
        11 从机地址
        05 功能码
        00 寄存器地址高字节
        AC 寄存器地址低字节
        FF 数据1高字节
        00 数据2低字节
        4E CRC校验高字节
        8B CRC校验低字节

        从机应答:
        11 从机地址
        05 功能码
        00 寄存器地址高字节
        AC 寄存器地址低字节
        FF 寄存器1高字节
        00 寄存器1低字节
        4E CRC校验高字节
        8B CRC校验低字节

        例子:
        01 05 10 01 FF 00   D93A   -- D01打开
        01 05 10 01 00 00   98CA   -- D01关闭

        01 05 10 02 FF 00   293A   -- D02打开
        01 05 10 02 00 00   68CA   -- D02关闭

        01 05 10 03 FF 00   78FA   -- D03打开
        01 05 10 03 00 00   390A   -- D03关闭
    */
    uint16_t reg;
    uint16_t value;
    uint8_t i;
    
    if (_mbs->rxCounter != 8)
    {
        _mbs->rspCode = RSP_ERR_VALUE;        /* 数据值域错误 */
    }

    reg = ((uint16_t)_mbs->rxBuf[2] << 8) | _mbs->rxBuf[3];  // 寄存器号
    value = ((uint16_t)_mbs->rxBuf[4] << 8) | _mbs->rxBuf[5]; // 数据

    if (value != 0x0000 && value != 0xff00)
    {
        _mbs->rspCode = RSP_ERR_VALUE;        // 数据值域错误
    }

    /* 如果改变的数据在寄存器范围内,则设置线圈状态 */
    if (reg <= REG_COILS_SIZE)
    {
        if(value)
        {
            mbs_hook_extract_coils(_mbs, reg, 0xff); // 设置状态寄存器
        }
        else
        {
            mbs_hook_extract_coils(_mbs, reg, 0x00); // 设置状态寄存器
        }
    }
    else
    {
        _mbs->rspCode = RSP_ERR_REG_ADDR;  // 寄存器地址错误
    }

    if (_mbs->rspCode == RSP_OK)  // 正确应答
    {
        for (i = 0; i < 6; i++)
        {
            _mbs->txBuf[i] = _mbs->rxBuf[i];
            _mbs->txLen++;
        }
    }
    else // 错误应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1] | 0x80;
        _mbs->txBuf[_mbs->txLen++] = _mbs->rspCode;
    }
    
    mbs_send(_mbs); // 发送应答
}

/**********************************************************************************************************
** Function name        :   mods_06h
** Descriptions         :   写单个寄存器
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mods_06h(mbs *_mbs)
{
    /*
        写保持寄存器。注意06指令只能操作单个保持寄存器，16指令可以设置单个或多个保持寄存器

        主机发送:
        11 从机地址
        06 功能码
        00 寄存器地址高字节
        01 寄存器地址低字节
        00 数据1高字节
        01 数据1低字节
        9A CRC校验高字节
        9B CRC校验低字节

        从机响应:
        11 从机地址
        06 功能码
        00 寄存器地址高字节
        01 寄存器地址低字节
        00 数据1高字节
        01 数据1低字节
        1B CRC校验高字节
        5A	CRC校验低字节
    */

    uint16_t reg;
    uint16_t value;
    uint16_t i;

    if (_mbs->rxCounter != 8)
    {
        _mbs->rspCode = RSP_ERR_VALUE;  // 数据值域错误
    }

    reg = ((uint16_t)_mbs->rxBuf[2] << 8) | _mbs->rxBuf[3];  // 寄存器号
    value = ((uint16_t)_mbs->rxBuf[4] << 8) | _mbs->rxBuf[5];  // 寄存器值
    
    /* 如果改变的数据在寄存器范围内,则写入寄存器数据 */
    if (reg <= REG_HOLDING_NREGS)
    {
        mbs_hook_extract_holding(_mbs, reg, value); // 写入寄存器数据/
    }
    else
    {
        _mbs->rspCode = RSP_ERR_REG_ADDR;  // 寄存器地址错误
    }

    if (_mbs->rspCode == RSP_OK)  // 正确应答
    {
        for (i = 0; i < 6; i++)
        {
            _mbs->txBuf[i] = _mbs->rxBuf[i];
            _mbs->txLen++;
        }
    }
    else // 错误应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1] | 0x80;
        _mbs->txBuf[_mbs->txLen++] = _mbs->rspCode;
    }
    
    mbs_send(_mbs); // 发送应答
}

/**********************************************************************************************************
** Function name        :   mods_0fh
** Descriptions         :   写多个单线圈
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mods_0fh(mbs *_mbs)
{
    /*
        主机发送:
        10 从机地址
        0F 功能码
        00 寄存器起始地址高字节
        00 寄存器起始地址低字节
        00 寄存器数量高字节
        10 寄存器数量低字节
        02 字节数
        FF 数据1高字节
        FF 数据1低字节
        23 CRC校验高字节
        C0 CRC校验低字节

        从机响应:
        10 从机地址
        0F 功能码
        00 寄存器地址高字节
        00 寄存器地址低字节
        00 数据1高字节
        10 数据1低字节
        57 CRC校验高字节
        46	CRC校验低字节
    */
    uint16_t reg_addr;
    uint16_t reg_num;
    uint8_t byte_num;
    uint8_t i;
    uint8_t value;

    if (_mbs->rxCounter  < 10)
    {
        _mbs->rspCode = RSP_ERR_VALUE;            // 数据值域错误
    }

    reg_addr = ((uint16_t)_mbs->rxBuf[2] << 8) | _mbs->rxBuf[3];  // 寄存器号
    reg_num = ((uint16_t)_mbs->rxBuf[4] << 8) | _mbs->rxBuf[5];     // 寄存器个数/
    byte_num = _mbs->rxBuf[6];                    /* 后面的数据体字节数 */
    
    if (byte_num > (reg_num / 8))
    {
        _mbs->rspCode = RSP_ERR_VALUE;            // 数据值域错误
    }
    
    for(i = 0; i < reg_num; i++)
    {
        value = _mbs->rxBuf[7 + i / 8];   /* 逐个取出寄存器字节数据 */
        
        /* 如果改变的数据在寄存器范围内,则设置线圈状态 */
        if (reg_addr <= REG_COILS_SIZE)
        {
            /* 判断字节数据的每一位 */
            if((value >> (i % 8)) & 0x01)
            {
                mbs_hook_extract_coils(_mbs, reg_addr, 0xff); // 设置状态寄存器
            }
            else
            {
                mbs_hook_extract_coils(_mbs, reg_addr, 0x00); // 设置状态寄存器
            }
            
            reg_addr++;  // 累加寄存器地址,直到读取完所有数据
        }
        else
        {
            _mbs->rspCode = RSP_ERR_REG_ADDR;  /* 寄存器地址错误 */
        }
    }
    
    if (_mbs->rspCode == RSP_OK)  // 正确应答
    {
        for (i = 0; i < 6; i++)
        {
            _mbs->txBuf[i] = _mbs->rxBuf[i];
            _mbs->txLen++;
        }
    }
    else // 错误应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1] | 0x80;
        _mbs->txBuf[_mbs->txLen++] = _mbs->rspCode;
    }
    
    mbs_send(_mbs); // 发送应答
}

/**********************************************************************************************************
** Function name        :   mods_10h
** Descriptions         :   连续写多个寄存器
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mods_10h(mbs *_mbs)
{
    /*
        主机发送:
        11 从机地址
        10 功能码
        00 寄存器起始地址高字节
        01 寄存器起始地址低字节
        00 寄存器数量高字节
        02 寄存器数量低字节
        04 字节数
        00 数据1高字节
        0A 数据1低字节
        01 数据2高字节
        02 数据2低字节
        C6 CRC校验高字节
        F0 CRC校验低字节

        从机响应:
        11 从机地址
        06 功能码
        00 寄存器地址高字节
        01 寄存器地址低字节
        00 数据1高字节
        01 数据1低字节
        1B CRC校验高字节
        5A	CRC校验低字节
    */
    uint16_t reg_addr;
    uint16_t reg_num;
    uint8_t byte_num;
    uint8_t i;
    uint16_t value;

    if (_mbs->rxCounter < 11)
    {
        _mbs->rspCode = RSP_ERR_VALUE; // 数据值域错误
    }

    reg_addr = ((uint16_t)_mbs->rxBuf[2] << 8) | _mbs->rxBuf[3]; // 寄存器号
    reg_num = ((uint16_t)_mbs->rxBuf[4] << 8) | _mbs->rxBuf[5]; // 寄存器号 /* 寄存器个数 */
    byte_num = _mbs->rxBuf[6]; // 后面的数据体字节数

    if (byte_num != 2 * reg_num)
    {
        _mbs->rspCode = RSP_ERR_VALUE; // 数据值域错误
    }
    
    for (i = 0; i < reg_num; i++)
    {
        value = ((uint16_t)_mbs->rxBuf[7 + 2 * i] << 8) | _mbs->rxBuf[8 + 2 * i];  /* 寄存器值 */
        
        /* 如果改变的数据在寄存器范围内,则写入寄存器数据 */
        if ((reg_addr <= REG_HOLDING_NREGS))
        {
            mbs_hook_extract_holding(_mbs, reg_addr, value);  /* 写入寄存器数据 */
            reg_addr++;  /* 累加寄存器地址,直到读取完所有数据 */
        }
        else
        {
            _mbs->rspCode = RSP_ERR_REG_ADDR;  /* 寄存器地址错误 */
        }
    }

    if (_mbs->rspCode == RSP_OK)  // 正确应答
    {
        for (i = 0; i < 6; i++)
        {
            _mbs->txBuf[i] = _mbs->rxBuf[i];
            _mbs->txLen++;
        }
    }
    else // 错误应答
    {
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
        _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1] | 0x80;
        _mbs->txBuf[_mbs->txLen++] = _mbs->rspCode;
    }
    
    mbs_send(_mbs); // 发送应答
}

/**********************************************************************************************************
** Function name        :   mbs_exec
** Descriptions         :   MODBUS回调函数处理
** parameters           :   pframe: 接收帧数据
**						:	len: 帧长度
** Returned value       :   无
***********************************************************************************************************/
void mbs_exec(mbs *_mbs)
{
    switch(_mbs->rxBuf[1])
    {
        case 1: mods_01h(_mbs); break;
        case 2: mods_01h(_mbs); break;
        case 3: mods_03h(_mbs); break;
        case 4: mods_03h(_mbs); break;
        case 5: mods_05h(_mbs); break;
        case 6: mods_06h(_mbs); break;
        case 15: mods_0fh(_mbs); break;
        case 16: mods_10h(_mbs); break;
        default:
            _mbs->rspCode = RSP_ERR_CMD;
            _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[0];
            _mbs->txBuf[_mbs->txLen++] = _mbs->rxBuf[1] | 0x80;
            _mbs->txBuf[_mbs->txLen++] = _mbs->rspCode;
            mbs_send(_mbs); // 发送应答
            break;
    }
}

/**********************************************************************************************************
** Function name        :   mbs_poll
** Descriptions         :   MODBUS状态轮训
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_poll(mbs *_mbs)
{
    switch(_mbs->state)
    {
        /*接收完一帧数据,开始进行校验*/
        case MBS_STATE_RX_CHECK:  //接收完成，对一帧数据进行检查
            if((_mbs->rxCounter >= MBS_RTU_MIN_SIZE) && (mb_crc16(_mbs->rxBuf, _mbs->rxCounter) == 0)) 	//接收的一帧数据正确
            {
                if(_mbs->rxBuf[0] == _mbs->slaveAddr)			//接收到的帧数据地址与从机地址相同
                {
                    _mbs->state = MBS_STATE_EXEC;
                }
                else
                {
                    _mbs->state = MBS_STATE_REC_ERR;
                }

            }
            else
            {
                _mbs->state = MBS_STATE_REC_ERR;
            }

            break;
            
        /*接收一帧数据出错*/
        case MBS_STATE_REC_ERR:
            _mbs->rxCounter = 0;
            _mbs->state = MBS_STATE_RX;
            break;

        /*确定接收正确执行回调*/
        case MBS_STATE_EXEC:      
            _mbs->txLen = 0; // 清除发送长度
            _mbs->rspCode = RSP_OK; // 置位应答码
            mbs_exec(_mbs); // 执行回调
            break;
    }
}

/**********************************************************************************************************
** Function name        :   mbh_timer3T5Isr
** Descriptions         :   modbus定时器中断处理
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_timer3T5Isr(mbs *_mbs)
{
    switch(_mbs->state)
    {
        case MBS_STATE_RX: //3.5T到,接收一帧完成
            _mbs->state = MBS_STATE_RX_CHECK;
            _mbs->timerDisable(&_mbs->htim); //关闭定时器
            break;
    }

}

/**********************************************************************************************************
** Function name        :   mbs_uartRxIsr
** Descriptions         :   modbus串口接收中断处理
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_uartRxIsr(mbs *_mbs)
{
    uint8_t ch;
    _mbs->getchar(&_mbs->huart, &ch);

    switch(_mbs->state)
    {
        case MBS_STATE_TX_END:
            _mbs->rxCounter = 0;
            _mbs->rxBuf[_mbs->rxCounter++] = ch;
            _mbs->state = MBS_STATE_RX;
            _mbs->timerEnable(&_mbs->htim);
            break;

        case MBS_STATE_RX:
            if(_mbs->rxCounter < MBS_RTU_MAX_SIZE)
            {
                _mbs->rxBuf[_mbs->rxCounter++] = ch;
            }

            _mbs->timerEnable(&_mbs->htim);
            break;

        default:
            _mbs->timerEnable(&_mbs->htim);
            break;
    }
}

/**********************************************************************************************************
** Function name        :   mbs_uartRxIsr
** Descriptions         :   modbus串口发送中断处理
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_uartTxIsr(mbs *_mbs)
{
    switch (_mbs->state)
    {
        case MBS_STATE_TX:
            if(_mbs->txCounter == _mbs->txLen) //全部发送完
            {
                _mbs->state = MBS_STATE_TX_END;
                _mbs->uartEnable(&_mbs->huart, 0, 1);  //disable tx,enable rx
                _mbs->timerEnable(&_mbs->htim);     //open timer
            }
            else
            {
                _mbs->putchar(&_mbs->huart, _mbs->txBuf[_mbs->txCounter++]);
            }

            break;

        case MBS_STATE_TX_END:
            _mbs->uartEnable(&_mbs->huart, 0, 1);  	 //disable tx,enable rx
            break;
        
        default: break;  	 //disable tx,enable rx   
    }
}

/********************************************** END OF FILE ***********************************************/
