/*************************************** Copyright (c)******************************************************
** File name            :   mb_hook.c
** Latest modified Date :   2025-06-20
** Latest Version       :   1.00
** Descriptions         :   modbus 主机回调函数处理
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
#include "mb_hook.h"
#include "SMD.h"
/** Public variables ------------------------------------------------------------------------------------*/

/**********************************************************************************************************
** Function name        :   mbs_hook_updata_holding
** Descriptions         :   MODBUS更新保存寄存器值
** parameters           :   _mbs: 从机结构体
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_updata_holding(mbs *_mbs)
{
    uint8_t i = 0;
    
    if(_mbs == &mbsUSB || _mbs == &mbsESP) // USB通信、ESP32通信 保存寄存器更新
    {
        for(i = 0; i < 8; i++)
        {
            _mbs->regHoldingBuf[SMD_1_AM_ADDR + i*10]  = SMD_AM_READ(i);
            _mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10]  = SMD_EN_READ(i);
            _mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10]  = SMD_DR_READ(i);
            _mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10]  = SMD_PU_DATA[i];
            _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] = SMD_ACC_DATA[i];
			_mbs->regHoldingBuf[SMD_1_SP_ADDR + i*10]  = (uint16_t) smd_freq_gradient[i].current_freq_int;
        }
        
        for(i = 0; i < 20; i++)
        {
            _mbs->regHoldingBuf[IN_1_ADDR + i] = IN_READ(i);
        }
        
        for(i = 0; i < 16; i++)
        {
            _mbs->regHoldingBuf[OUT_1_ADDR + i] = OUT_READ(i);
        }
        
        for(i = 0; i < 16; i++)
        {
            _mbs->regHoldingBuf[ADC_1_ADDR + i] = mbsSTM.regHoldingBuf[i];
        }
    }
    else if(_mbs == &mbsSTM) // STM32通信 保存寄存器更新
    {
    }
}

/**********************************************************************************************************
** Function name        :   mbs_hook_extract_holding
** Descriptions         :   MODBUS提取保存寄存器值
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val)
{
    uint8_t i = 0;
    
    _mbs->regHoldingBuf[_reg] = _val;
    
    if(_mbs == &mbsUSB || _mbs == &mbsESP)
    {
        for(i = 0; i < 8; i++)
        {
            if(_mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10] != SMD_EN_READ(i))
            {
                SMD_EN(i, _mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10]);
            }
            
            if(_mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10] != SMD_DR_READ(i))
            {
                SMD_DR(i, _mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10]);
            }
			if(_mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10] != SMD_PU_DATA[i] ||\
				_mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] != SMD_ACC_DATA[i])
			{
				SMD_PU_DATA[i] =  _mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10];
				SMD_ACC_DATA[i] =  _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10];
			    SMD_PWM_SetFreqGradient((SMD_Channel)i,SMD_PU_DATA[i],SMD_ACC_DATA[i]);
			}
        }
        
        for(i = 0; i < 16; i++)
        {
            if(_mbs->regHoldingBuf[OUT_1_ADDR + i] != OUT_READ(i))
            {
                OUT(i, _mbs->regHoldingBuf[OUT_1_ADDR + i]);
            }
        }
        
        for(i = 0; i < 5; i++)
        {
            if(_mbs->regHoldingBuf[EC_CLEAR_1_ADDR + i] != 0)
            {
                mbsSTM.regHoldingBuf[i + 11] = _mbs->regHoldingBuf[EC_CLEAR_1_ADDR + i];
            }
        }
        
        switch(_reg)
        {
            default: break;
        }
    }
    else if(_mbs == &mbsSTM)
    {
        switch(_reg)
        {
            default: break;
        }
    }
    
    
}

/**********************************************************************************************************
** Function name        :   mbs_hook_updata_coils
** Descriptions         :   MODBUS更新线圈状态
** parameters           :   _mbs: 从机结构体
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_updata_coils(mbs *_mbs)
{
    _mbs->regCoilsBuf[0] |= 0x01;
}

/**********************************************************************************************************
** Function name        :   mbs_hook_extract_coils
** Descriptions         :   MODBUS提取线圈状态
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_extract_coils(mbs *_mbs, uint16_t _reg, uint8_t _val)
{
    if(_val)
    {
        _mbs->regCoilsBuf[_reg / 8] |= 1 << (_reg % 8);  /* 设置状态寄存器 */
    }
    else
    {
        _mbs->regCoilsBuf[_reg / 8] &= ~(1 << (_reg % 8));  /* 设置状态寄存器 */
    }
}
/********************************************** END OF FILE ***********************************************/
