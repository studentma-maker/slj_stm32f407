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

/**********************************************************************************************************
** Function name        :   mbs_hook_updata_holding
** Descriptions         :   MODBUS更新保存寄存器值（读取时刷新寄存器镜像）
** parameters           :   _mbs: 从机结构体
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_updata_holding(mbs *_mbs)
{
    uint8_t i = 0;
    
    if(_mbs == &mbsUSB || _mbs == &mbsESP)
    {
        for(i = 0; i < 8; i++)
        {
            _mbs->regHoldingBuf[SMD_1_AM_ADDR   + i*10] = SMD_AM_READ(i);
            _mbs->regHoldingBuf[SMD_1_DR_ADDR   + i*10] = SMD_DR_READ(i);
            _mbs->regHoldingBuf[SMD_1_ACC_ADDR  + i*10] = SMD_ACC_DATA[i];
            _mbs->regHoldingBuf[SMD_1_JRK_ADDR  + i*10] = (uint16_t)(SMD_JERK_DATA[i] > 65535u ? 65535u : SMD_JERK_DATA[i]);
            /* STEP：步数控制触发寄存器（只写，回读始终为 0） */
            _mbs->regHoldingBuf[SMD_1_STEP_ADDR + i*10] = 0;
            _mbs->regHoldingBuf[SMD_1_PU_ADDR   + i*10] = SMD_PU_DATA[i];
            /* SP：实时速度，主机可读取当前运行频率 */
            _mbs->regHoldingBuf[SMD_1_SP_ADDR   + i*10] = (uint16_t)smd_freq_gradient[i].current_freq_int;
        }
        
        for(i = 0; i < 20; i++)
        {
            _mbs->regHoldingBuf[IN_1_ADDR + i] = IN_READ(i);
        }
        
        for(i = 0; i < 16; i++)
        {
            _mbs->regHoldingBuf[OUT_1_ADDR + i] = OUT_READ(i);
        }
        
        for(i = 0; i < 6; i++)
        {
            _mbs->regHoldingBuf[ADC_1_ADDR + i] = mbsSTM.regHoldingBuf[i];
        }

        _mbs->regHoldingBuf[GRIPPER_CUR_STEPS] = MotorCurStepsU[MOTOR_GripperMove];
        _mbs->regHoldingBuf[FBACK_CUR_STEPS]   = MotorCurStepsU[MOTOR_FBack];
        _mbs->regHoldingBuf[SYS_TO_ORIGIN] = g_sysToOrigin;
    }
    else if(_mbs == &mbsSTM)
    {
        /* STM32从机侧无需额外处理 */
    }
}

/**********************************************************************************************************
** Function name        :   mbs_hook_extract_holding
** Descriptions         :   MODBUS提取保存寄存器值（写入时触发执行）
** parameters           :   _mbs: 从机结构体
**                      :   _reg: 写入的寄存器地址
**                      :   _val: 写入的值
** Returned value       :   无
**
** 寄存器写入行为说明（v2.0 步数控制重构）：
**
**  DR   (偏移1) ：设置 dir_change 标志（S曲线换向保护），步数模式下忽略
**  ACC  (偏移2) ：更新最大加速度，所有模式下均可写入，限幅 SMD_ACC_MAX_MIN~MAX
**  JRK  (偏移3) ：更新 jerk 值（Hz/s²），写 0 自动恢复 SMD_JERK_DEFAULT
**  PU   (偏移4) ：非步数模式→启动S曲线；步数模式→更新最大脉冲频率
**  STEP (偏移5) ：写入目标步数→进入步数模式
**  SP   (偏移6) ：写 0=急停；写 N>0=直接跳变到 N Hz
**
***********************************************************************************************************/
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val)
{
    uint8_t i = 0;

    if((_val == WRITE_HOLDING_V) || (g_sysToOrigin != toOriginSuccess))
        return;

    _mbs->regHoldingBuf[_reg] = _val;

    if(_mbs == &mbsUSB || _mbs == &mbsESP)
    {
        for(i = 0; i < 8; i++)
        {
            /* --- 方向控制（偏移1）：步数模式下忽略，方向由步数差自动决定 --- */
            if(!g_motorStepsCtl[i].is_running)
            {
                if(_mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10] != SMD_DR_READ(i))
                {
                    smd_freq_gradient[i].dir_change = 1;
                    smd_freq_gradient[i].dir_state  = SMD_DIR_NORMAL;
                }
            }

            /* --- JRK写入（偏移3）：所有模式下均可更新 --- */
            if(_mbs->regHoldingBuf[SMD_1_JRK_ADDR + i*10] != SMD_JERK_DATA[i])
            {
                uint32_t jrk_val = (uint32_t)_mbs->regHoldingBuf[SMD_1_JRK_ADDR + i*10];
                if(jrk_val == 0)                 jrk_val = SMD_JERK_DEFAULT;
                if(jrk_val < SMD_JERK_MIN)       jrk_val = SMD_JERK_MIN;
                if(jrk_val > SMD_JERK_MAX)       jrk_val = SMD_JERK_MAX;
                SMD_JERK_DATA[i] = jrk_val;
                _mbs->regHoldingBuf[SMD_1_JRK_ADDR + i*10] = (uint16_t)(jrk_val > 65535u ? 65535u : jrk_val);
            }

            /* --- ACC_MAX写入（偏移2）：所有模式下均可更新 --- */
            if(_mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] != SMD_ACC_DATA[i])
            {
                uint16_t acc_val = _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10];
                if(acc_val < SMD_ACC_MAX_MIN) acc_val = SMD_ACC_MAX_MIN;
                if(acc_val > SMD_ACC_MAX_MAX) acc_val = (uint16_t)SMD_ACC_MAX_MAX;
                SMD_ACC_DATA[i] = acc_val;
                _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] = acc_val;
            }

            /* --- STEP写入（偏移4）：进入步数控制模式 ---
             * 在 PU（偏移5）之前处理：多寄存器连续写入时先触发步数模式，
             * 后续 PU 写入自动走「仅更新最大频率」分支。 */
            if(_reg == (SMD_1_STEP_ADDR + i*10))
            {
                uint16_t target_steps = _mbs->regHoldingBuf[SMD_1_STEP_ADDR + i*10];
                g_motorStepsCtl[i].targetSteps = target_steps;
                g_motorStepsCtl[i].is_running  = 1;
                g_motorStepsCtl[i].braking     = 0;
            }

            /* --- PU写入（偏移5）：步数模式仅更新最大频率，非步数模式启动S曲线 ---
             * STEP 在 PU 之前处理，故此处 g_motorStepsCtl[i].is_running 已正确反映
             * 当前是否处于步数模式，无需额外 _reg 匹配。 */
            if(_mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10] != SMD_PU_DATA[i])
            {
                uint16_t pu_freq = _mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10];
                if(pu_freq < SMD_PWM_FREQ_MIN) pu_freq = SMD_PWM_FREQ_MIN;
                if(pu_freq > SMD_PWM_FREQ_MAX) pu_freq = (uint16_t)SMD_PWM_FREQ_MAX;
                SMD_PU_DATA[i]  = pu_freq;
                _mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10] = pu_freq;

                if (!g_motorStepsCtl[i].is_running)
                {
                    /* 非步数模式：启动S曲线渐变 */
                    SMD_PWM_SetFreqGradient((SMD_Channel)i, SMD_PU_DATA[i], SMD_ACC_DATA[i]);
                }
                /* 步数模式：仅更新 SMD_PU_DATA[i]，SMD_MotorStepsCtl 在下个中断取用 */
            }

            /* --- SP写入（偏移6）：直接跳变 / 急停 --- */
            if(_reg == (SMD_1_SP_ADDR + i*10))
            {
                uint16_t sp_cmd = _mbs->regHoldingBuf[SMD_1_SP_ADDR + i*10];

                if(sp_cmd == 0)
                {
                    /* 急停：硬件停止，清除运动状态 */
                    SMD_PWM_Stop((SMD_Channel)i);
                    SMD_PU_DATA[i] = SMD_PWM_FREQ_MIN;
                    smd_freq_gradient[i].v_c              = 0.0f;
                    smd_freq_gradient[i].v_n              = 0.0f;
                    smd_freq_gradient[i].a_c              = 0.0f;
                    smd_freq_gradient[i].current_freq_int = SMD_PWM_FREQ_MIN;
                    smd_freq_gradient[i].is_running       = 0;
                    g_motorStepsCtl[i].is_running         = 0;
                    g_motorStepsCtl[i].braking            = 0;
                }
                else if (sp_cmd > 1)
                {
                    /* 直接跳变到指定频率（限幅），不经过S曲线 */
                    uint16_t target = sp_cmd;
                    if (target < (uint16_t)SMD_PWM_FREQ_MIN) target = (uint16_t)SMD_PWM_FREQ_MIN;
                    if (target > (uint16_t)SMD_PWM_FREQ_MAX) target = (uint16_t)SMD_PWM_FREQ_MAX;
                    SMD_PWM_SetFreq((SMD_Channel)i, target);
                    smd_freq_gradient[i].v_c  = (float)target;
                    smd_freq_gradient[i].v_n  = (float)target;
                    smd_freq_gradient[i].a_c  = 0.0f;
                    smd_freq_gradient[i].is_running = 0;
                    SMD_PU_DATA[i] = target;
                    g_motorStepsCtl[i].is_running = 0;
                    g_motorStepsCtl[i].braking = 0;
                }
            }

            /* --- 全部急停寄存器 --- */
            if(_reg == STOP_ALL_MOTOR_ADDR)
            {
                uint16_t s_cmd = _mbs->regHoldingBuf[STOP_ALL_MOTOR_ADDR];
                if(s_cmd == 1)
                {
                    SMD_PWM_Stop((SMD_Channel)i);
                    SMD_PU_DATA[i] = SMD_PWM_FREQ_MIN;
                    smd_freq_gradient[i].v_c              = 0.0f;
                    smd_freq_gradient[i].v_n              = 0.0f;
                    smd_freq_gradient[i].a_c              = 0.0f;
                    smd_freq_gradient[i].current_freq_int = SMD_PWM_FREQ_MIN;
                    smd_freq_gradient[i].is_running       = 0;
                    g_motorStepsCtl[i].is_running         = 0;
                    g_motorStepsCtl[i].braking            = 0;
                }
            }
        }
        /* --- 输出控制 --- */
        for(i = 0; i < 16; i++)
        {
			if(i == RELAY_MOTOR_1 && _mbs->regHoldingBuf[OUT_1_ADDR + i] == 1)
			{
				OUT(i + 1, _mbs->regHoldingBuf[OUT_1_ADDR + i + 1]);
			}
			if(i == RELAY_MOTOR_1_a && _mbs->regHoldingBuf[OUT_1_ADDR + i] == 1)
			{
				OUT(i + 1, _mbs->regHoldingBuf[OUT_1_ADDR + i + 1]);
			}
            if(_mbs->regHoldingBuf[OUT_1_ADDR + i] != OUT_READ(i))
            {
                OUT(i, _mbs->regHoldingBuf[OUT_1_ADDR + i]);
            }
        }

        /* --- 编码器清零（转发给STM从机） --- */
        for(i = 0; i < 5; i++)
        {
            if(_mbs->regHoldingBuf[EC_CLEAR_1_ADDR + i] != 0)
            {
                mbsSTM.regHoldingBuf[i + 11] = _mbs->regHoldingBuf[EC_CLEAR_1_ADDR + i];
            }
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
        _mbs->regCoilsBuf[_reg / 8] |= 1 << (_reg % 8);
    else
        _mbs->regCoilsBuf[_reg / 8] &= ~(1 << (_reg % 8));
}

/********************************************** END OF FILE ***********************************************/
