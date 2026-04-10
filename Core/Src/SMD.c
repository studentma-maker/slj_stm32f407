#include "SMD.h"
#include "main.h"
#include <math.h>
#include <stdint.h>

/* ========================= 全局变量 ========================= */
uint32_t num = 0;

/* 普通PWM通道定时器句柄 */
TIM_HandleTypeDef htim2_smd;
TIM_HandleTypeDef htim9_smd;
TIM_HandleTypeDef htim5_smd;
TIM_HandleTypeDef htim13_smd;
TIM_HandleTypeDef htim14_smd;
TIM_HandleTypeDef htim3_smd;

/* 高级定时器句柄（互补通道） */
TIM_HandleTypeDef htim8_smd;
TIM_HandleTypeDef htim1_smd;

/* TIM10句柄（1ms调度中断） */
TIM_HandleTypeDef htim10;

/* Modbus可读写的电机参数 */
/* 1、电机驱动目标脉冲频率 */
uint16_t SMD_PU_DATA[8]   = {
	SMD_PWM_FREQ_MIN,  SMD_PWM_FREQ_MIN,  
	SMD_PWM_FREQ_MIN,  SMD_PWM_FREQ_MIN,  
	SMD_PWM_FREQ_MIN,  SMD_PWM_FREQ_MIN,  
	SMD_PWM_FREQ_MIN,  SMD_PWM_FREQ_MIN
};
/* 2、电机加速度最大值 */
uint16_t SMD_ACC_DATA[8] = {
    SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT,
    SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT,
    SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT,
    SMD_ACC_MAX_DEFAULT, SMD_ACC_MAX_DEFAULT
};
/* 3、电机默认加加速度 */
uint16_t SMD_JERK_DATA[8] = {
	SMD_JERK_DEFAULT, SMD_JERK_DEFAULT, 
	SMD_JERK_DEFAULT, SMD_JERK_DEFAULT, 
	SMD_JERK_DEFAULT, SMD_JERK_DEFAULT, 
	SMD_JERK_DEFAULT, SMD_JERK_DEFAULT
};
uint16_t GripperCurStepsU = 0; 
static float GripperCurStepsF = 0; 

/**
 * @brief  8路PWM通道的S曲线运动状态
 * @note   v_c/v_n 单位 Hz（浮点），a_c 单位 Hz/s，jerk 字段为运行时动态值
 *         jerk 的初始值和用户设定值由 SMD_JERK_DATA[] 管理，
 *         中断每次迭代从 SMD_JERK_DATA[ch] 取最新值，无需手动同步结构体。
 */
SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX] = {
    /* v_c   a_c   v_n   jerk  dir_change dir_state  freq_int  is_running */
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH0
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH1
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH2
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH3
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH4
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH5
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH6
    {0.0f, 0.0f, 0.0f, 0.0f,  0, SMD_DIR_NORMAL, 1, 0},  // CH7
};

/* ========================= HAL底层GPIO初始化回调 ========================= */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (tim_pwmHandle->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_5;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_0;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM9)
    {
        __HAL_RCC_TIM9_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_2;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM13)
    {
        __HAL_RCC_TIM13_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_6;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM14)
    {
        __HAL_RCC_TIM14_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (tim_pwmHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ========================= 内部：高级定时器初始化 ========================= */
static HAL_StatusTypeDef SMD_Init_HighTimer(TIM_HandleTypeDef *htim, uint32_t psc, uint32_t arr, uint32_t tim_channel)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim->Init.Prescaler         = psc;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = arr;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(htim) != HAL_OK) return HAL_ERROR;

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, tim_channel) != HAL_OK) return HAL_ERROR;

    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;

    if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/* ========================= 内部：普通定时器初始化 ========================= */
static HAL_StatusTypeDef SMD_Init_NormalTimer(TIM_HandleTypeDef *htim, uint32_t psc, uint32_t arr, uint32_t tim_channel)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim->Init.Prescaler         = psc;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = arr;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(htim) != HAL_OK) return HAL_ERROR;

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, tim_channel) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/* ========================= 内部：频率转PSC/ARR ========================= */
/**
 * @brief  根据目标频率和通道计算PSC和ARR
 * @note   APB2（TIM1/8/9）挂168MHz，APB1（其余）挂84MHz
 *         CH1=TIM9, CH3=TIM8, CH6=TIM1 → 168MHz bus
 */
static void SMD_Calc_PSC_ARR(uint32_t freq, uint32_t *psc, uint32_t *arr, int ch)
{
    if (freq < SMD_PWM_FREQ_MIN) freq = SMD_PWM_FREQ_MIN;
    if (freq > SMD_PWM_FREQ_MAX) freq = SMD_PWM_FREQ_MAX;

    if (ch == 1 || ch == 3 || ch == 6)
        *psc = (168000000 / SMD_COUNTER_FREQ) - 1;  // 167
    else
        *psc = (84000000 / SMD_COUNTER_FREQ) - 1;   // 83

    *arr = (SMD_COUNTER_FREQ / freq) - 1;
}

/* ========================= 内部：直接写入硬件定时器频率（不停PWM） ========================= */
/**
 * @brief  仅更新PSC/ARR寄存器，不停止PWM，适合中断内调用
 * @note   利用 ARR 预装载，修改在下一个周期生效，无毛刺
 */
static void SMD_ApplyFreqToHW(SMD_Channel ch, uint32_t freq_int)
{
    uint32_t psc, arr;
    SMD_Calc_PSC_ARR(freq_int, &psc, &arr, (int)ch);

    /* 直接写寄存器，避免在中断内调用完整HAL_TIM_PWM_Init带来的开销 */
    switch (ch)
    {
        case SMD_CH0:
            htim2_smd.Instance->PSC = psc;
            htim2_smd.Instance->ARR = arr;
            htim2_smd.Instance->CCR2 = arr / 2;   // 保持50%占空比
            //htim2_smd.Instance->EGR  = TIM_EGR_UG; // 立即更新PSC生效
            break;
        case SMD_CH1:
            htim9_smd.Instance->PSC = psc;
            htim9_smd.Instance->ARR = arr;
            htim9_smd.Instance->CCR1 = arr / 2;
            //htim9_smd.Instance->EGR  = TIM_EGR_UG;
            break;
        case SMD_CH2:
            htim5_smd.Instance->PSC = psc;
            htim5_smd.Instance->ARR = arr;
            htim5_smd.Instance->CCR4 = arr / 2;
            //htim5_smd.Instance->EGR  = TIM_EGR_UG;
            break;
        case SMD_CH3:
            htim8_smd.Instance->PSC = psc;
            htim8_smd.Instance->ARR = arr;
            htim8_smd.Instance->CCR1 = arr / 2;
            //htim8_smd.Instance->EGR  = TIM_EGR_UG;
            break;
        case SMD_CH4:
            htim13_smd.Instance->PSC = psc;
            htim13_smd.Instance->ARR = arr;
            htim13_smd.Instance->CCR1 = arr / 2;
            //htim13_smd.Instance->EGR  = TIM_EGR_UG;
            break;
        case SMD_CH5:
            htim14_smd.Instance->PSC = psc;
            htim14_smd.Instance->ARR = arr;
            htim14_smd.Instance->CCR1 = arr / 2;
            //htim14_smd.Instance->EGR  = TIM_EGR_UG;
            break;
        case SMD_CH6:
            htim1_smd.Instance->PSC = psc;
            htim1_smd.Instance->ARR = arr;
            htim1_smd.Instance->CCR2 = arr / 2;
            //htim1_smd.Instance->EGR  = TIM_EGR_UG;
            break;
        case SMD_CH7:
            htim3_smd.Instance->PSC = psc;
            htim3_smd.Instance->ARR = arr;
            htim3_smd.Instance->CCR4 = arr / 2;
            //htim3_smd.Instance->EGR  = TIM_EGR_UG;
            break;
        default: break;
    }

    smd_freq_gradient[ch].current_freq_int = freq_int;
    if (ch == MOTOR_GripperMove)
    {
        GripperCurStepsF += (float)freq_int * SMD_UPDATE_DT_s;

        uint16_t whole = (uint16_t)GripperCurStepsF;
        if (whole > 0)
        {
            GripperCurStepsF -= (float)whole;

            /* 读取方向引脚：1→正转累加，0→反转累减 */
            if (SMD_DR_READ(ch))
                GripperCurStepsU += whole;
            else{
                if(GripperCurStepsU < whole) GripperCurStepsU = 0;
                else GripperCurStepsU -= whole;
            }
        }
    }
}

/* ========================= 内部：S曲线速度更新（移植自ESP32 update_velocity） ========================= */
/**
 * @brief  S曲线加减速核心：根据当前速度与目标速度的差距动态调整加速度
 * @param  m:         通道状态指针
 * @param  delta_v:   |v_n - v_c|，剩余速度差（Hz）
 * @param  accel_max: 本通道最大加速度（Hz/s），来自 SMD_ACC_DATA[ch]
 * @param  jerk:      本通道jerk（Hz/s²），来自 SMD_JERK_DATA[ch]，用户可自定义
 * @note   移植自 motor_smctl.c update_velocity()
 *         加速阶段：以 jerk 速率持续增大加速度，上限为 accel_max
 *         减速阶段：当剩余速差不足以平滑制动时，以计算出的 jerk 降低加速度
 */
static void SMD_UpdateVelocity(SMD_Freq_Gradient *m, float delta_v, float accel_max, float jerk)
{
    /* 判断是否需要全力加速：剩余速差 > 能在当前加速度下平滑制动的最小距离 */
    float decel_dist = 0.5f * m->a_c * m->a_c / jerk;
    uint8_t need_max_accel = (delta_v > decel_dist) ? 1 : 0;

    if (need_max_accel)
    {
        /* 加速段：以用户设定的jerk增大加速度 */
        m->a_c += jerk * SMD_UPDATE_DT_s;
        if (m->a_c > accel_max)
            m->a_c = accel_max;
    }
    else
    {
        /* 减速段：计算此刻所需jerk，降低加速度 */
        if (delta_v > 0.01f)
            m->jerk = 0.5f * m->a_c * m->a_c / delta_v;
        else
            m->jerk = jerk;

        m->a_c -= m->jerk * SMD_UPDATE_DT_s;
        if (m->a_c < 0.0f)
            m->a_c = 0.0f;
    }
}

/* ========================= PWM初始化 ========================= */
HAL_StatusTypeDef SMD_PWM_Init(SMD_Channel ch)
{
    if (ch >= SMD_CH_MAX) return HAL_ERROR;

    uint32_t psc, arr;
    /* 初始化时以1Hz计算PSC，后续由中断动态调频 */
    SMD_Calc_PSC_ARR(1, &psc, &arr, (int)ch);

    switch (ch)
    {
        case SMD_CH0: htim2_smd.Instance  = TIM2;  return SMD_Init_NormalTimer(&htim2_smd,  psc, arr, TIM_CHANNEL_2);
        case SMD_CH1: htim9_smd.Instance  = TIM9;  return SMD_Init_NormalTimer(&htim9_smd,  psc, arr, TIM_CHANNEL_1);
        case SMD_CH2: htim5_smd.Instance  = TIM5;  return SMD_Init_NormalTimer(&htim5_smd,  psc, arr, TIM_CHANNEL_4);
        case SMD_CH3: htim8_smd.Instance  = TIM8;  return SMD_Init_HighTimer  (&htim8_smd,  psc, arr, TIM_CHANNEL_1);
        case SMD_CH4: htim13_smd.Instance = TIM13; return SMD_Init_NormalTimer(&htim13_smd, psc, arr, TIM_CHANNEL_1);
        case SMD_CH5: htim14_smd.Instance = TIM14; return SMD_Init_NormalTimer(&htim14_smd, psc, arr, TIM_CHANNEL_1);
        case SMD_CH6: htim1_smd.Instance  = TIM1;  return SMD_Init_HighTimer  (&htim1_smd,  psc, arr, TIM_CHANNEL_2);
        case SMD_CH7: htim3_smd.Instance  = TIM3;  return SMD_Init_NormalTimer(&htim3_smd,  psc, arr, TIM_CHANNEL_4);
        default:      return HAL_ERROR;
    }
}

HAL_StatusTypeDef SMD_PWM_InitAll(void)
{
    HAL_StatusTypeDef ret = HAL_OK;

    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
        if (SMD_PWM_Init((SMD_Channel)ch) != HAL_OK)
        {
            ret = HAL_ERROR;
            break;
        }
    }

    if (SMD_TIM10_Init() != HAL_OK)
        ret = HAL_ERROR;

    return ret;
}

/* ========================= PWM启停 ========================= */
HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch)
{
    switch (ch)
    {
        case SMD_CH0: return HAL_TIM_PWM_Start(&htim2_smd,   TIM_CHANNEL_2);
        case SMD_CH1: return HAL_TIM_PWM_Start(&htim9_smd,   TIM_CHANNEL_1);
        case SMD_CH2: return HAL_TIM_PWM_Start(&htim5_smd,   TIM_CHANNEL_4);
        case SMD_CH3: return HAL_TIMEx_PWMN_Start(&htim8_smd,  TIM_CHANNEL_1);
        case SMD_CH4: return HAL_TIM_PWM_Start(&htim13_smd,  TIM_CHANNEL_1);
        case SMD_CH5: return HAL_TIM_PWM_Start(&htim14_smd,  TIM_CHANNEL_1);
        case SMD_CH6: return HAL_TIMEx_PWMN_Start(&htim1_smd,  TIM_CHANNEL_2);
        case SMD_CH7: return HAL_TIM_PWM_Start(&htim3_smd,   TIM_CHANNEL_4);
        default:      return HAL_ERROR;
    }
}

HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch)
{
    switch (ch)
    {
        case SMD_CH0: return HAL_TIM_PWM_Stop(&htim2_smd,   TIM_CHANNEL_2);
        case SMD_CH1: return HAL_TIM_PWM_Stop(&htim9_smd,   TIM_CHANNEL_1);
        case SMD_CH2: return HAL_TIM_PWM_Stop(&htim5_smd,   TIM_CHANNEL_4);
        case SMD_CH3: return HAL_TIMEx_PWMN_Stop(&htim8_smd,  TIM_CHANNEL_1);
        case SMD_CH4: return HAL_TIM_PWM_Stop(&htim13_smd,  TIM_CHANNEL_1);
        case SMD_CH5: return HAL_TIM_PWM_Stop(&htim14_smd,  TIM_CHANNEL_1);
        case SMD_CH6: return HAL_TIMEx_PWMN_Stop(&htim1_smd,  TIM_CHANNEL_2);
        case SMD_CH7: return HAL_TIM_PWM_Stop(&htim3_smd,   TIM_CHANNEL_4);
        default:      return HAL_ERROR;
    }
}

HAL_StatusTypeDef SMD_PWM_StartAll(void)
{
    HAL_StatusTypeDef ret = HAL_OK;
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
        if (SMD_PWM_Start((SMD_Channel)ch) != HAL_OK) { ret = HAL_ERROR; break; }
    return ret;
}

HAL_StatusTypeDef SMD_PWM_StopAll(void)
{
    HAL_StatusTypeDef ret = HAL_OK;
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
        if (SMD_PWM_Stop((SMD_Channel)ch) != HAL_OK) { ret = HAL_ERROR; break; }
    return ret;
}

/* ========================= 占空比设置 ========================= */
void SMD_PWM_SetDuty(SMD_Channel ch, uint32_t duty)
{
    if (ch >= SMD_CH_MAX) return;
	if (smd_freq_gradient[ch].current_freq_int == 0) return;
    uint32_t max_duty = SMD_COUNTER_FREQ / smd_freq_gradient[ch].current_freq_int;
    if (duty > max_duty) duty = max_duty;

    switch (ch)
    {
        case SMD_CH0: __HAL_TIM_SET_COMPARE(&htim2_smd,   TIM_CHANNEL_2, duty); break;
        case SMD_CH1: __HAL_TIM_SET_COMPARE(&htim9_smd,   TIM_CHANNEL_1, duty); break;
        case SMD_CH2: __HAL_TIM_SET_COMPARE(&htim5_smd,   TIM_CHANNEL_4, duty); break;
        case SMD_CH3: __HAL_TIM_SET_COMPARE(&htim8_smd,   TIM_CHANNEL_1, duty); break;
        case SMD_CH4: __HAL_TIM_SET_COMPARE(&htim13_smd,  TIM_CHANNEL_1, duty); break;
        case SMD_CH5: __HAL_TIM_SET_COMPARE(&htim14_smd,  TIM_CHANNEL_1, duty); break;
        case SMD_CH6: __HAL_TIM_SET_COMPARE(&htim1_smd,   TIM_CHANNEL_2, duty); break;
        case SMD_CH7: __HAL_TIM_SET_COMPARE(&htim3_smd,   TIM_CHANNEL_4, duty); break;
        default: break;
    }
}

void SMD_PWM_SetDutyPercent(SMD_Channel ch, float percent)
{
    if (ch >= SMD_CH_MAX) return;
	if (smd_freq_gradient[ch].current_freq_int == 0) return;
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    uint32_t max_duty = SMD_COUNTER_FREQ / smd_freq_gradient[ch].current_freq_int;
    SMD_PWM_SetDuty(ch, (uint32_t)((percent / 100.0f) * (float)max_duty));
}

/* ========================= 频率直接设置（阻塞） ========================= */
HAL_StatusTypeDef SMD_PWM_SetFreq(SMD_Channel ch, uint32_t freq)
{
    if (ch >= SMD_CH_MAX || freq < SMD_PWM_FREQ_MIN || freq > SMD_PWM_FREQ_MAX)
        return HAL_ERROR;

    uint32_t psc, arr;
    SMD_Calc_PSC_ARR(freq, &psc, &arr, (int)ch);

    HAL_StatusTypeDef ret = SMD_PWM_Stop(ch);
    if (ret != HAL_OK) return ret;

    switch (ch)
    {
        case SMD_CH0: htim2_smd.Init.Prescaler  = psc; htim2_smd.Init.Period  = arr; ret = HAL_TIM_PWM_Init(&htim2_smd);  break;
        case SMD_CH1: htim9_smd.Init.Prescaler  = psc; htim9_smd.Init.Period  = arr; ret = HAL_TIM_PWM_Init(&htim9_smd);  break;
        case SMD_CH2: htim5_smd.Init.Prescaler  = psc; htim5_smd.Init.Period  = arr; ret = HAL_TIM_PWM_Init(&htim5_smd);  break;
        case SMD_CH3: htim8_smd.Init.Prescaler  = psc; htim8_smd.Init.Period  = arr; ret = HAL_TIM_PWM_Init(&htim8_smd);  break;
        case SMD_CH4: htim13_smd.Init.Prescaler = psc; htim13_smd.Init.Period = arr; ret = HAL_TIM_PWM_Init(&htim13_smd); break;
        case SMD_CH5: htim14_smd.Init.Prescaler = psc; htim14_smd.Init.Period = arr; ret = HAL_TIM_PWM_Init(&htim14_smd); break;
        case SMD_CH6: htim1_smd.Init.Prescaler  = psc; htim1_smd.Init.Period  = arr; ret = HAL_TIM_PWM_Init(&htim1_smd);  break;
        case SMD_CH7: htim3_smd.Init.Prescaler  = psc; htim3_smd.Init.Period  = arr; ret = HAL_TIM_PWM_Init(&htim3_smd);  break;
        default: return HAL_ERROR;
    }
    if (ret != HAL_OK) return ret;

    smd_freq_gradient[ch].v_c = (float)freq;
    smd_freq_gradient[ch].current_freq_int = freq;

    return SMD_PWM_Start(ch);
}

/* ========================= S曲线渐变接口（对外） ========================= */
/**
 * @brief  设置目标频率和最大加速度，启动S曲线渐变
 * @param  ch:          通道
 * @param  target_freq: 目标频率（Hz），范围 SMD_PWM_FREQ_MIN ~ SMD_PWM_FREQ_MAX
 * @param  accel:       最大加速度 ACC_MAX（Hz/s），范围 SMD_ACC_MAX_MIN ~ SMD_ACC_MAX_MAX
 *                      S曲线加速过程：a_c 以 SMD_JERK_DATA[ch] 从0增大到 accel，
 *                      到达 accel 后匀加速，接近目标时对称减速至0。
 */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t accel)
{
    if (ch >= SMD_CH_MAX)                                              return HAL_ERROR;
    if (target_freq < SMD_PWM_FREQ_MIN || target_freq > SMD_PWM_FREQ_MAX) return HAL_ERROR;
    if (accel < SMD_ACC_MAX_MIN || accel > SMD_ACC_MAX_MAX)            return HAL_ERROR;

    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];

    m->v_n        = (float)target_freq;
    SMD_ACC_DATA[ch] = (uint16_t)accel; // 存储 ACC_MAX，中断里取用

	if(target_freq > SMD_PWM_FREQ_MIN) SMD_PWM_Start(ch);
    m->is_running = 1;

    return HAL_OK;
}

void SMD_PWM_StopFreqGradient(SMD_Channel ch)
{
    if (ch >= SMD_CH_MAX) return;
    smd_freq_gradient[ch].is_running = 0;
    smd_freq_gradient[ch].a_c        = 0.0f;
}

/* ========================= TIM10 1ms中断初始化 ========================= */
HAL_StatusTypeDef SMD_TIM10_Init(void)
{
    __HAL_RCC_TIM10_CLK_ENABLE();

    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 167;   // 168MHz / 168 = 1MHz
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    //htim10.Init.Period            = 999;   // 1MHz / 1000 = 1kHz → 1ms
    htim10.Init.Period            = (SMD_COUNTER_FREQ / (SMD_UPDATE_DT_ms * 1000)) - 1;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) return HAL_ERROR;

    __HAL_TIM_ENABLE_IT(&htim10, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0); // 优先级1，低于Modbus串口(0)
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/* ========================= TIM10 1ms中断服务函数（S曲线调度） ========================= */
/**
 * @brief  TIM1更新/TIM10全局中断，1ms周期
 *
 * 移植自ESP32 ideal_motor_smctl_task()，去掉FreeRTOS，改为中断驱动。
 * 每1ms对每个通道执行一次S曲线迭代：
 *
 *  ┌─────────────────────────────────────────────────────────┐
 *  │ 1. 检查换向标志 dir_change                               │
 *  │    → 若需换向且 v_c != 0：强制 v_n = 0（先减速）         │
 *  │    → 若需换向且 v_c ≈ 0 ：切换 DR 引脚，清除换向标志      │
 *  │ 2. 计算剩余速差 delta_v = |v_n - v_c|                   │
 *  │ 3. 若已到达目标：v_c = v_n，停止加速度                   │
 *  │ 4. 调用 SMD_UpdateVelocity() 更新 a_c（S曲线核心）       │
 *  │ 5. 积分速度：v_c ± a_c * dt                             │
 *  │ 6. 边界限幅，写入硬件                                    │
 *  └─────────────────────────────────────────────────────────┘
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);

    num++;

    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
        SMD_Freq_Gradient *m = &smd_freq_gradient[ch];

        if (!m->is_running) continue;

        /* ---- 1. 换向状态机 ---- */
        if (m->dir_change)
        {
            if (m->dir_state == SMD_DIR_NORMAL)
            {
                /* 进入换向：开始减速到0 */
                m->dir_state = SMD_DIR_DECEL;
            }

            if (m->dir_state == SMD_DIR_DECEL)
            {
                /* 减速过程中强制目标为0 */
                float delta_v = m->v_c;  // v_c > 0

                if (delta_v <= 1.0f)
                {
                    /* 已到0，停止PWM，切换方向引脚 */
                    m->v_c = 0.0f;
                    m->a_c = 0.0f;
                    SMD_PWM_Stop((SMD_Channel)ch);

                    /* 切换方向：翻转DR引脚 */
                    uint8_t cur_dir = (uint8_t)SMD_DR_READ(ch);
                    SMD_DR(ch, !cur_dir);

                    /* HAL_Delay在中断中不可用，给2ms裕量靠多次迭代完成 */
                    m->dir_state  = SMD_DIR_WAIT;
                    m->dir_change = 0;

                    SMD_PWM_Start((SMD_Channel)ch);
                    continue;  // 本ms不再加速
                }

                /* 仍在减速，以S曲线减速 */
                SMD_UpdateVelocity(m, delta_v, (float)SMD_ACC_DATA[ch], (float)SMD_JERK_DATA[ch]);
                m->v_c -= m->a_c * SMD_UPDATE_DT_s;
                if (m->v_c < 0.0f) m->v_c = 0.0f;

                uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
                if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
                SMD_ApplyFreqToHW((SMD_Channel)ch, freq_int);
                continue;
            }

            if (m->dir_state == SMD_DIR_WAIT)
            {
                /* GPIO切换后等1个周期再恢复正常运动 */
                m->dir_state = SMD_DIR_NORMAL;
                continue;
            }
        }

        /* ---- 2. 正常S曲线运动 ---- */
        float v_target = m->v_n;
        float delta_v  = v_target - m->v_c;
        float abs_dv   = (delta_v >= 0.0f) ? delta_v : -delta_v;
        float accel_max = (float)SMD_ACC_DATA[ch];
        float jerk_val  = (float)SMD_JERK_DATA[ch];

        /* 到达目标 */
        if (abs_dv <= 0.5f)
        {
            m->v_c = v_target;
            m->a_c = 0.0f;

            /* 目标为 SMD_PWM_FREQ_MIN（1Hz）表示"减速停止"语义：
             * 停止 PWM 硬件输出，避免电机仍以 1Hz（1步/秒）低速蠕动。
             * is_running 置 0，中断不再驱动此通道，直到下次写 PU 重新激活。 */
            if (v_target <= (float)SMD_PWM_FREQ_MIN)
            {
                SMD_PWM_Stop((SMD_Channel)ch);
                m->v_c              = 0.0f;
                m->a_c              = 0.0f;
                m->is_running       = 0;
                continue;
            }

            uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
            if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
            if (freq_int > SMD_PWM_FREQ_MAX) freq_int = SMD_PWM_FREQ_MAX;
            SMD_ApplyFreqToHW((SMD_Channel)ch, freq_int);
            continue;
        }

        /* S曲线：更新加速度 */
        SMD_UpdateVelocity(m, abs_dv, accel_max, jerk_val);

        /* 积分速度 */
        if (delta_v > 0.0f)
            m->v_c += m->a_c * SMD_UPDATE_DT_s;
        else
            m->v_c -= m->a_c * SMD_UPDATE_DT_s;

        /* 边界限幅 */
        if (m->v_c < (float)SMD_PWM_FREQ_MIN) m->v_c = (float)SMD_PWM_FREQ_MIN;
        if (m->v_c > (float)SMD_PWM_FREQ_MAX) m->v_c = (float)SMD_PWM_FREQ_MAX;

        /* 写入硬件 */
        uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
        if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
        if (freq_int > SMD_PWM_FREQ_MAX) freq_int = SMD_PWM_FREQ_MAX;
        SMD_ApplyFreqToHW((SMD_Channel)ch, freq_int);
    }
}
