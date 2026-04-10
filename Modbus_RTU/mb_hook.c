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
            _mbs->regHoldingBuf[SMD_1_AM_ADDR  + i*10] = SMD_AM_READ(i);
            _mbs->regHoldingBuf[SMD_1_EN_ADDR  + i*10] = SMD_EN_READ(i);
            _mbs->regHoldingBuf[SMD_1_DR_ADDR  + i*10] = SMD_DR_READ(i);
            _mbs->regHoldingBuf[SMD_1_PU_ADDR  + i*10] = SMD_PU_DATA[i];
            _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] = SMD_ACC_DATA[i];
            /* JRK：当前jerk设定值，主机可读写（偏移5） */
            _mbs->regHoldingBuf[SMD_1_JRK_ADDR + i*10] = (uint16_t)(SMD_JERK_DATA[i] > 65535u ? 65535u : SMD_JERK_DATA[i]);
            /* SP：实时速度，主机可读取当前运行频率（偏移6） */
            _mbs->regHoldingBuf[SMD_1_SP_ADDR  + i*10] = (uint16_t)smd_freq_gradient[i].current_freq_int;
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

        _mbs->regHoldingBuf[GRIPPER_CUR_STEPS] = GripperCurStepsU;
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
** 寄存器写入行为说明：
**
**  EN  (偏移1) ：直接控制使能 GPIO
**  DR  (偏移2) ：设置 dir_change 标志，中断里先减速到0，再切换 GPIO，再加速（S曲线换向保护）
**  PU  (偏移3) ：更新目标频率，重新启动 S 曲线渐变
**  ACC (偏移4) ：更新最大加速度，重新启动 S 曲线渐变
**  JRK (偏移5) ：更新本通道 jerk 值（Hz/s²），写 0 自动恢复默认值 SMD_JERK_DEFAULT
**  SP  (偏移6) ：写 0 = 急停（立即停止PWM，清除运动状态）；写 N>0 = 直接跳变到 N Hz（绕过S曲线）
**
***********************************************************************************************************/
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val)
{
    uint8_t i = 0;
    
    if(_val == WRITE_HOLDING_V)
        return;
    
    _mbs->regHoldingBuf[_reg] = _val;
    
    if(_mbs == &mbsUSB || _mbs == &mbsESP)
    {
        switch(_reg)
        {
            case GRIPPER_TARGET_STEPS:
                g_gripperStepsCtl.targetSteps = _mbs->regHoldingBuf[GRIPPER_TARGET_STEPS];
                g_gripperStepsCtl.is_running = 1;
                return;
            default: break;
        }
        for(i = 0; i < 8; i++)
        {
            if(i == MOTOR_GripperMove && g_gripperStepsCtl.is_running) continue;
        	/* --- 一次性写入多个寄存器的值，顺序要求：先jerk 和acc_max 最后是pu --- */
            /* --- 使能控制 --- */
            if(_mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10] != SMD_EN_READ(i))
            {
                SMD_EN(i, _mbs->regHoldingBuf[SMD_1_EN_ADDR + i*10]);
            }
            
            /* --- 方向控制（S曲线换向） --- */
            if(_mbs->regHoldingBuf[SMD_1_DR_ADDR + i*10] != SMD_DR_READ(i))
            {
                /*
                 * 不直接写GPIO，而是设置换向标志。
                 * TIM10中断检测到 dir_change=1 后，会：
                 *   1. 将 v_n 临时置0，减速
                 *   2. v_c 到0时翻转 DR 引脚
                 *   3. 恢复 v_n = SMD_PU_DATA[i]，继续加速
                 *
                 * 注意：寄存器镜像先写入，GPIO切换由中断完成。
                 */
                smd_freq_gradient[i].dir_change = 1;
                smd_freq_gradient[i].dir_state  = SMD_DIR_NORMAL; // 由中断推进状态
                /* 同步寄存器镜像（GPIO实际切换在中断完成后才生效，
                   此处仅记录"期望方向"，SMD_DR_READ在切换前后会有短暂不一致，属正常） */
            }
			/* --- JRK写入：用户自定义jerk（偏移5）---
             * 写入范围 SMD_JERK_MIN ~ SMD_JERK_MAX，超出则限幅。
             * 写 0 视为无效，自动恢复 SMD_JERK_DEFAULT。
             * 新值在下一个 1ms 中断迭代时立即生效，无需重启渐变。
             */
            if(_mbs->regHoldingBuf[SMD_1_JRK_ADDR + i*10] != SMD_JERK_DATA[i])
            {
                uint32_t jrk_val = (uint32_t)_mbs->regHoldingBuf[SMD_1_JRK_ADDR + i*10];
                if(jrk_val == 0)                 jrk_val = SMD_JERK_DEFAULT;
                if(jrk_val < SMD_JERK_MIN)       jrk_val = SMD_JERK_MIN;
                if(jrk_val > SMD_JERK_MAX)       jrk_val = SMD_JERK_MAX;
                SMD_JERK_DATA[i] = jrk_val;
                _mbs->regHoldingBuf[SMD_1_JRK_ADDR + i*10] = (uint16_t)(jrk_val > 65535u ? 65535u : jrk_val);
            }
			/* --- ACC_MAX写入：用户自定义acc_max（偏移4）---
             * 写入范围 SMD_ACC_MAX_MIN ~ SMD_ACC_MAX_MAX，超出则限幅。
             */
			if(_mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] != SMD_ACC_DATA[i])
			{
                uint16_t acc_val = _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10];
                if(acc_val < SMD_ACC_MAX_MIN) acc_val = SMD_ACC_MAX_MIN;
                if(acc_val > SMD_ACC_MAX_MAX) acc_val = (uint16_t)SMD_ACC_MAX_MAX;
                SMD_ACC_DATA[i] = acc_val;
                _mbs->regHoldingBuf[SMD_1_ACC_ADDR + i*10] = acc_val;
			}
            /* --- 目标频率或加速度最大值变化：重新启动S曲线 ---
             * 写入时做范围限幅，超出则钳位到边界值。
             */
            if(_mbs->regHoldingBuf[SMD_1_PU_ADDR  + i*10] != SMD_PU_DATA[i])
            {
            	uint16_t pu_freq = _mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10];
				if(pu_freq < SMD_PWM_FREQ_MIN) pu_freq = SMD_PWM_FREQ_MIN;
                if(pu_freq > SMD_PWM_FREQ_MAX) pu_freq = (uint16_t)SMD_PWM_FREQ_MAX;
                SMD_PU_DATA[i]  = pu_freq;
				_mbs->regHoldingBuf[SMD_1_PU_ADDR + i*10] = pu_freq;
                SMD_PWM_SetFreqGradient((SMD_Channel)i, SMD_PU_DATA[i], SMD_ACC_DATA[i]);
            }
            /* --- SP写入：直接跳变 / 急停（偏移6）---
             *
             *  写 SP = 0   → 急停：立即 SMD_PWM_Stop()，清除速度和加速度状态，
             *                is_running 置 0，TIM10 中断不再驱动该通道。
             *                PU 寄存器保留原值，下次恢复只需重新写 PU 即可重启渐变，
             *                无需额外操作。
             *
             *  写 SP = N>0 → 直接跳变：绕过S曲线，硬件立即切换到 N Hz，
             *                同时清零 a_c（消除跳变前残留加速度），
             *                v_c 同步更新为 N，后续 PU 渐变以此为起点。
             *                N 超出范围自动限幅。
             *
             *  SP 是"一次性命令寄存器"：执行后下次 updata 上报的值回到实际
             *  current_freq_int，主机不应持续重复写同一值。
             */
            if(_reg == (SMD_1_SP_ADDR + i*10))
            {
                uint16_t sp_cmd = _mbs->regHoldingBuf[SMD_1_SP_ADDR + i*10];

                if(sp_cmd == 0)
                {
                    /* 急停：硬件停止，清除运动状态，保留PU目标值 */
                    SMD_PWM_Stop((SMD_Channel)i);
					SMD_PU_DATA[i] 						  = 1u;
                    smd_freq_gradient[i].v_c              = 0.0f;
                    smd_freq_gradient[i].v_n              = 0.0f;
                    smd_freq_gradient[i].a_c              = 0.0f;
                    smd_freq_gradient[i].current_freq_int = 1u; // 最小有效值，防止除零
                    smd_freq_gradient[i].is_running       = 0;
                    /* 注意：SMD_PU_DATA[i] 和对应寄存器保留不变。
                     * 下次向 PU 寄存器写入任意值，或重启 SetFreqGradient，
                     * 电机将从 0 重新加速到目标频率。 */
                }
                else
                {
                    /* 直接跳变到指定频率（限幅），清零残留加速度 */
					// 暂时去掉 跳到指定速度 只有急停，写入其他值无效
                    //uint16_t target = sp_cmd;
                    //if(target < (uint16_t)SMD_PWM_FREQ_MIN) target = (uint16_t)SMD_PWM_FREQ_MIN;
                    //if(target > (uint16_t)SMD_PWM_FREQ_MAX) target = (uint16_t)SMD_PWM_FREQ_MAX;
                    //SMD_PWM_SetFreq((SMD_Channel)i, target);
                    //smd_freq_gradient[i].a_c = 0.0f; // 消除跳变前残留加速度
                    /* v_c 已在 SMD_PWM_SetFreq 内同步为 target，后续PU渐变以此为起点 */
                }
            }
            /* ---写入全部急停寄存器,电机全部停止---
             *
             */
            if(_reg == STOP_ALL_MOTOR_ADDR)
            {
                uint16_t s_cmd = _mbs->regHoldingBuf[STOP_ALL_MOTOR_ADDR];
                if(s_cmd == 1)
                {
                    SMD_PWM_Stop((SMD_Channel)i);
					SMD_PU_DATA[i] 						  = 1u;
                    smd_freq_gradient[i].v_c              = 0.0f;
                    smd_freq_gradient[i].v_n              = 0.0f;
                    smd_freq_gradient[i].a_c              = 0.0f;
                    smd_freq_gradient[i].current_freq_int = 1u; // 最小有效值，防止除零
                    smd_freq_gradient[i].is_running       = 0;
                }
            }
        }
        /* --- 输出控制 --- */
        for(i = 0; i < 16; i++)
        {
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
