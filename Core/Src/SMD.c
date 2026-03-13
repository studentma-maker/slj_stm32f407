#include "SMD.h"
#include "main.h"  // 引入Error_Handler声明
#include <math.h>  // 浮点运算必备头文件
#include <stdint.h> // 标准整型定义

/* ========================= 全局变量定义（带初始化说明） ========================= */
uint32_t num = 0;
// 普通PWM通道定时器句柄（每个句柄对应一个定时器）
TIM_HandleTypeDef htim2_smd;
TIM_HandleTypeDef htim9_smd;
TIM_HandleTypeDef htim5_smd;
TIM_HandleTypeDef htim13_smd;
TIM_HandleTypeDef htim14_smd;
TIM_HandleTypeDef htim3_smd;

// 高级定时器句柄（互补通道专用）
TIM_HandleTypeDef htim8_smd;
TIM_HandleTypeDef htim1_smd;

// TIM10定时器句柄（1ms中断核心，复用测试通过的句柄名）
TIM_HandleTypeDef htim10;
uint16_t SMD_PU_DATA[8] = {1,1,1,1,1,1,1,1};
uint16_t SMD_ACC_DATA[8] = {100,100,100,100,100,100,100,100};
/**
 * @brief 8路PWM通道渐变参数初始化
 * @note  1. 初始频率200Hz（浮点型200.0f，整型200）
 *        2. 渐变状态默认停止，步长默认0
 */
SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX] = {
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH0: PA1-TIM2_CH2
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH1: PA2-TIM9_CH1
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH2: PA3-TIM5_CH4
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH3: PA5-TIM8_CH1N
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH4: PA6-TIM13_CH1
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH5: PA7-TIM14_CH1
    {200.0f, 200.0f, 0.0f, 0, 200}, // CH6: PB0-TIM1_CH2N
    {200.0f, 200.0f, 0.0f, 0, 200}  // CH7: PB1-TIM3_CH4
};

/* ========================= HAL底层GPIO初始化回调（带硬件配置说明） ========================= */
/**
 * @brief PWM定时器底层GPIO初始化回调（HAL自动调用）
 * @param tim_pwmHandle: 定时器句柄指针
 * @note  1. 每个定时器对应GPIO的时钟使能+引脚配置
 *        2. 复用功能严格匹配芯片手册，不可错误
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // TIM8初始化（CH3: PA5-TIM8_CH1N）
    if (tim_pwmHandle->Instance == TIM8)
    {
        // 1. 使能TIM8和GPIOA时钟
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // 2. PA5配置为TIM8_CH1N（AF3复用功能）
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;    // 推挽复用输出（PWM必备）
        GPIO_InitStruct.Pull = GPIO_NOPULL;        // PWM输出无需上下拉
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 高速模式（匹配PWM频率）
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8; // 复用功能3-TIM8（芯片手册规定）
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    // TIM1初始化（CH6: PB0-TIM1_CH2N）
    else if (tim_pwmHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1; // 复用功能1-TIM1
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    // TIM2初始化（CH0: PA1-TIM2_CH2）
    else if (tim_pwmHandle->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // 复用功能1-TIM2
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    // TIM9初始化（CH1: PA2-TIM9_CH1）
    else if (tim_pwmHandle->Instance == TIM9)
    {
        __HAL_RCC_TIM9_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM9; // 复用功能3-TIM9
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    // TIM5初始化（CH2: PA3-TIM5_CH4）
    else if (tim_pwmHandle->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5; // 复用功能2-TIM5
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    // TIM13初始化（CH4: PA6-TIM13_CH1）
    else if (tim_pwmHandle->Instance == TIM13)
    {
        __HAL_RCC_TIM13_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13; // 复用功能9-TIM13
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    // TIM14初始化（CH5: PA7-TIM14_CH1）
    else if (tim_pwmHandle->Instance == TIM14)
    {
        __HAL_RCC_TIM14_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM14; // 复用功能9-TIM14
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    // TIM3初始化（CH7: PB1-TIM3_CH4）
    else if (tim_pwmHandle->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // 复用功能2-TIM3
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ========================= 高级定时器初始化（带死区配置说明） ========================= */
/**
 * @brief 高级定时器初始化（TIM1/TIM8，带互补通道）
 * @param htim: 定时器句柄指针
 * @param psc: 预分频值
 * @param arr: 自动重装载值
 * @param tim_channel: 定时器通道
 * @retval HAL_StatusTypeDef: 初始化结果
 * @note   1. 高级定时器必须配置死区/断路功能，否则无法输出PWM
 *         2. PWM模式1：CNT < CCR时输出高电平
 */
static HAL_StatusTypeDef SMD_Init_HighTimer(TIM_HandleTypeDef *htim, uint32_t psc, uint32_t arr, uint32_t tim_channel)
{
    TIM_OC_InitTypeDef sConfigOC = {0};            // PWM输出配置结构体
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0}; // 死区/断路配置

    // 1. 定时器基础配置
    htim->Init.Prescaler = psc;                    // 预分频值（分频后到1MHz）
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;   // 向上计数（PWM标准模式）
    htim->Init.Period = arr;                       // 自动重装载值（决定频率）
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟不分频
    htim->Init.RepetitionCounter = 0;              // 重复计数器（高级定时器特有，0=无重复）
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 使能ARR预装载（频率修改立即生效）

    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
        return HAL_ERROR; // 初始化失败返回错误
    }

    // 2. PWM通道配置
    sConfigOC.OCMode = TIM_OCMODE_PWM1;            // PWM模式1（常用模式）
    sConfigOC.Pulse = 0;                           // 初始占空比0（CCR值）
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;    // 主通道极性高
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;  // 互补通道极性高
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;     // 关闭快速模式（普通PWM无需）
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; // 空闲状态复位（低电平）
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补通道空闲状态复位

    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, tim_channel) != HAL_OK)
    {
        return HAL_ERROR; // 通道配置失败返回错误
    }

    // 3. 死区/断路配置（高级定时器必须配置，否则禁止输出）
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;          // 关闭断路功能（无硬件保护需求）
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;   // 断路极性低（未使用）
    sBreakDeadTimeConfig.BreakFilter = 0;                         // 断路滤波器关闭（未使用）
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; // 关闭自动输出（手动控制）
    sBreakDeadTimeConfig.DeadTime = 0;                            // 死区时间0（无互补通道延时需求）
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;           // 关闭锁定级别（可修改配置）

    if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig) != HAL_OK)
    {
        return HAL_ERROR; // 死区配置失败返回错误
    }

    return HAL_OK; // 初始化成功
}

/* ========================= 普通定时器初始化（简化版） ========================= */
/**
 * @brief 普通定时器初始化（TIM2/3/5/9/13/14）
 * @param htim: 定时器句柄指针
 * @param psc: 预分频值
 * @param arr: 自动重装载值
 * @param tim_channel: 定时器通道
 * @retval HAL_StatusTypeDef: 初始化结果
 * @note   普通定时器无需死区配置，流程简化
 */
static HAL_StatusTypeDef SMD_Init_NormalTimer(TIM_HandleTypeDef *htim, uint32_t psc, uint32_t arr, uint32_t tim_channel)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // 1. 定时器基础配置（同高级定时器）
    htim->Init.Prescaler = psc;                   
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;   
    htim->Init.Period = arr;                       
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 

    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 2. PWM通道配置（无互补通道）
    sConfigOC.OCMode = TIM_OCMODE_PWM1;            
    sConfigOC.Pulse = 0;                           
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;    
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;     

    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, tim_channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ========================= 频率转PSC/ARR计算函数（核心计算逻辑） ========================= */
/**
 * @brief 根据目标频率计算定时器PSC和ARR值
 * @param freq: 目标频率（整型，1~4000Hz）
 * @param psc: 输出参数，预分频值
 * @param arr: 输出参数，自动重装载值
 * @note   1. 系统时钟84MHz，目标计数器频率1MHz → PSC = 84-1 = 83
 *         2. ARR = (1MHz / 目标频率) - 1 → 决定PWM周期
 *         3. 自动做频率边界保护，避免非法值
 */
static void SMD_Calc_PSC_ARR(uint32_t freq, uint32_t *psc, uint32_t *arr ,int ch)
{
    // 频率边界保护：强制限定在1~4000Hz
    if (freq < SMD_PWM_FREQ_MIN)
    {
        freq = SMD_PWM_FREQ_MIN;
    }
    else if (freq > SMD_PWM_FREQ_MAX)
    {
        freq = SMD_PWM_FREQ_MAX;
    }
    if(ch == 1 || ch == 3 || ch == 6)
	{
	  // 计算PSC：168MHz / 1MHz - 1 = 167
      *psc = (168000000 / SMD_COUNTER_FREQ) - 1;
	}
	else
	{
	  // 计算PSC：84MHz / 1MHz - 1 = 83
      *psc = (84000000 / SMD_COUNTER_FREQ) - 1;
	}
    
    // 计算ARR：1MHz / 目标频率 - 1（例：200Hz → 1e6/200 -1 = 4999）
    *arr = (SMD_COUNTER_FREQ / freq) - 1;
}

/* ========================= PWM基础功能实现（带调用逻辑说明） ========================= */
HAL_StatusTypeDef SMD_PWM_Init(SMD_Channel ch)
{
    // 参数校验：通道必须小于最大值（避免数组越界）
    if (ch >= SMD_CH_MAX)
    {
        return HAL_ERROR;
    }

    uint32_t psc, arr;
    // 初始频率1Hz，计算对应的PSC和ARR
    SMD_Calc_PSC_ARR(1, &psc, &arr ,ch);

    // 根据通道选择对应的定时器初始化
    switch(ch)
    {
        case SMD_CH0: // PA1-TIM2_CH2
            htim2_smd.Instance = TIM2;
            return SMD_Init_NormalTimer(&htim2_smd, psc, arr, TIM_CHANNEL_2);
        case SMD_CH1: // PA2-TIM9_CH1
            htim9_smd.Instance = TIM9;
            return SMD_Init_NormalTimer(&htim9_smd, psc, arr, TIM_CHANNEL_1);
        case SMD_CH2: // PA3-TIM5_CH4
            htim5_smd.Instance = TIM5;
            return SMD_Init_NormalTimer(&htim5_smd, psc, arr, TIM_CHANNEL_4);
        case SMD_CH3: // PA5-TIM8_CH1N
            htim8_smd.Instance = TIM8;
            return SMD_Init_HighTimer(&htim8_smd, psc, arr, TIM_CHANNEL_1);
        case SMD_CH4: // PA6-TIM13_CH1
            htim13_smd.Instance = TIM13;
            return SMD_Init_NormalTimer(&htim13_smd, psc, arr, TIM_CHANNEL_1);
        case SMD_CH5: // PA7-TIM14_CH1
            htim14_smd.Instance = TIM14;
            return SMD_Init_NormalTimer(&htim14_smd, psc, arr, TIM_CHANNEL_1);
        case SMD_CH6: // PB0-TIM1_CH2N
            htim1_smd.Instance = TIM1;
            return SMD_Init_HighTimer(&htim1_smd, psc, arr, TIM_CHANNEL_2);
        case SMD_CH7: // PB1-TIM3_CH4
            htim3_smd.Instance = TIM3;
            return SMD_Init_NormalTimer(&htim3_smd, psc, arr, TIM_CHANNEL_4);
        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef SMD_PWM_InitAll(void)
{
    HAL_StatusTypeDef ret = HAL_OK;

    // 遍历初始化所有8路通道
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
        if (SMD_PWM_Init((SMD_Channel)ch) != HAL_OK)
        {
            ret = HAL_ERROR; // 有一个通道失败则标记整体失败
            break;
        }
    }

    // 初始化TIM10 1ms中断（渐变驱动核心）
    if (SMD_TIM10_Init() != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    return ret;
}

HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch)
{
    // 根据通道类型选择启动函数（普通/PWMN）
    switch(ch)
    {
        case SMD_CH0:
            return HAL_TIM_PWM_Start(&htim2_smd, TIM_CHANNEL_2);
        case SMD_CH1:
            return HAL_TIM_PWM_Start(&htim9_smd, TIM_CHANNEL_1);
        case SMD_CH2:
            return HAL_TIM_PWM_Start(&htim5_smd, TIM_CHANNEL_4);
        case SMD_CH3: // 互补通道需启动PWMN
            return HAL_TIMEx_PWMN_Start(&htim8_smd, TIM_CHANNEL_1);
        case SMD_CH4:
            return HAL_TIM_PWM_Start(&htim13_smd, TIM_CHANNEL_1);
        case SMD_CH5:
            return HAL_TIM_PWM_Start(&htim14_smd, TIM_CHANNEL_1);
        case SMD_CH6: // 互补通道需启动PWMN
            return HAL_TIMEx_PWMN_Start(&htim1_smd, TIM_CHANNEL_2);
        case SMD_CH7:
            return HAL_TIM_PWM_Start(&htim3_smd, TIM_CHANNEL_4);
        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch)
{
    // 根据通道类型选择停止函数
    switch(ch)
    {
        case SMD_CH0:
            return HAL_TIM_PWM_Stop(&htim2_smd, TIM_CHANNEL_2);
        case SMD_CH1:
            return HAL_TIM_PWM_Stop(&htim9_smd, TIM_CHANNEL_1);
        case SMD_CH2:
            return HAL_TIM_PWM_Stop(&htim5_smd, TIM_CHANNEL_4);
        case SMD_CH3: // 互补通道需停止PWMN
            return HAL_TIMEx_PWMN_Stop(&htim8_smd, TIM_CHANNEL_1);
        case SMD_CH4:
            return HAL_TIM_PWM_Stop(&htim13_smd, TIM_CHANNEL_1);
        case SMD_CH5:
            return HAL_TIM_PWM_Stop(&htim14_smd, TIM_CHANNEL_1);
        case SMD_CH6: // 互补通道需停止PWMN
            return HAL_TIMEx_PWMN_Stop(&htim1_smd, TIM_CHANNEL_2);
        case SMD_CH7:
            return HAL_TIM_PWM_Stop(&htim3_smd, TIM_CHANNEL_4);
        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef SMD_PWM_StartAll(void)
{
    HAL_StatusTypeDef ret = HAL_OK;

    // 遍历启动所有通道
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
        if (SMD_PWM_Start((SMD_Channel)ch) != HAL_OK)
        {
            ret = HAL_ERROR;
            break;
        }
    }

    return ret;
}

HAL_StatusTypeDef SMD_PWM_StopAll(void)
{
    HAL_StatusTypeDef ret = HAL_OK;

    // 遍历停止所有通道
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
        if (SMD_PWM_Stop((SMD_Channel)ch) != HAL_OK)
        {
            ret = HAL_ERROR;
            break;
        }
    }

    return ret;
}

void SMD_PWM_SetDuty(SMD_Channel ch, uint32_t duty)
{
    // 参数校验：避免数组越界
    if (ch >= SMD_CH_MAX)
    {
        return;
    }

    // 计算当前频率下的最大计数值（1MHz / 当前整型频率）
    uint32_t max_duty = SMD_COUNTER_FREQ / smd_freq_gradient[ch].current_freq_int;
    // 边界保护：占空比不能超过最大值
    if (duty > max_duty)
    {
        duty = max_duty;
    }

    // 设置对应通道的CCR值（修改占空比）
    switch(ch)
    {
        case SMD_CH0:
            __HAL_TIM_SET_COMPARE(&htim2_smd, TIM_CHANNEL_2, duty);
            break;
        case SMD_CH1:
            __HAL_TIM_SET_COMPARE(&htim9_smd, TIM_CHANNEL_1, duty);
            break;
        case SMD_CH2:
            __HAL_TIM_SET_COMPARE(&htim5_smd, TIM_CHANNEL_4, duty);
            break;
        case SMD_CH3: // 互补通道占空比由主通道CCR控制
            __HAL_TIM_SET_COMPARE(&htim8_smd, TIM_CHANNEL_1, duty);
            break;
        case SMD_CH4:
            __HAL_TIM_SET_COMPARE(&htim13_smd, TIM_CHANNEL_1, duty);
            break;
        case SMD_CH5:
            __HAL_TIM_SET_COMPARE(&htim14_smd, TIM_CHANNEL_1, duty);
            break;
        case SMD_CH6: // 互补通道占空比由主通道CCR控制
            __HAL_TIM_SET_COMPARE(&htim1_smd, TIM_CHANNEL_2, duty);
            break;
        case SMD_CH7:
            __HAL_TIM_SET_COMPARE(&htim3_smd, TIM_CHANNEL_4, duty);
            break;
        default:
            break;
    }
}

void SMD_PWM_SetDutyPercent(SMD_Channel ch, float percent)
{
    // 参数校验：避免数组越界
    if (ch >= SMD_CH_MAX)
    {
        return;
    }

    // 百分比边界保护
    if (percent < 0.0f)
    {
        percent = 0.0f;
    }
    else if (percent > 100.0f)
    {
        percent = 100.0f;
    }

    // 计算最大计数值
    uint32_t max_duty = SMD_COUNTER_FREQ / smd_freq_gradient[ch].current_freq_int;
    // 浮点运算转换百分比到计数值（保证精度）
    uint32_t duty = (uint32_t)((percent / 100.0f) * (float)max_duty);
    // 调用计数值版本的占空比设置函数
    SMD_PWM_SetDuty(ch, duty);
}

/* ========================= 频率设置与渐变功能（核心浮点计算） ========================= */
HAL_StatusTypeDef SMD_PWM_SetFreq(SMD_Channel ch, uint32_t freq)
{
    // 1. 严格参数校验：通道合法 + 频率在限定范围
    if (ch >= SMD_CH_MAX || freq < SMD_PWM_FREQ_MIN || freq > SMD_PWM_FREQ_MAX)
    {
        return HAL_ERROR;
    }

    // 2. 计算对应频率的PSC和ARR
    uint32_t psc, arr;
    SMD_Calc_PSC_ARR(freq, &psc, &arr ,ch);

    // 3. 先停止PWM输出（避免配置过程中输出异常）
    HAL_StatusTypeDef ret = SMD_PWM_Stop(ch);
    if (ret != HAL_OK)
    {
        return ret;
    }

    // 4. 重新配置定时器的PSC和ARR（修改频率）
    switch(ch)
    {
        case SMD_CH0:
            htim2_smd.Init.Prescaler = psc;
            htim2_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim2_smd);
            break;
        case SMD_CH1:
            htim9_smd.Init.Prescaler = psc;
            htim9_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim9_smd);
            break;
        case SMD_CH2:
            htim5_smd.Init.Prescaler = psc;
            htim5_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim5_smd);
            break;
        case SMD_CH3:
            htim8_smd.Init.Prescaler = psc;
            htim8_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim8_smd);
            break;
        case SMD_CH4:
            htim13_smd.Init.Prescaler = psc;
            htim13_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim13_smd);
            break;
        case SMD_CH5:
            htim14_smd.Init.Prescaler = psc;
            htim14_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim14_smd);
            break;
        case SMD_CH6:
            htim1_smd.Init.Prescaler = psc;
            htim1_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim1_smd);
            break;
        case SMD_CH7:
            htim3_smd.Init.Prescaler = psc;
            htim3_smd.Init.Period = arr;
            ret = HAL_TIM_PWM_Init(&htim3_smd);
            break;
        default:
            return HAL_ERROR;
    }

    // 配置失败直接返回
    if (ret != HAL_OK)
    {
        return ret;
    }

    // 5. 同步更新渐变参数（浮点+整型）
    smd_freq_gradient[ch].current_freq_float = (float)freq; // 浮点型同步
    smd_freq_gradient[ch].current_freq_int = freq;          // 整型同步

    // 6. 重启PWM输出
    return SMD_PWM_Start(ch);
}

HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t speed)
{
    // 1. 严格参数校验（所有参数必须在限定范围）
    if (ch >= SMD_CH_MAX ||                          // 通道非法
        target_freq < SMD_PWM_FREQ_MIN || target_freq > SMD_PWM_FREQ_MAX || // 频率超界
        speed < SMD_GRADIENT_MIN || speed > SMD_GRADIENT_MAX) // 速度超界
    {
        return HAL_ERROR;
    }

    // 2. 停止当前通道的渐变（避免叠加）
    SMD_PWM_StopFreqGradient(ch);

    // 3. 浮点型计算每ms步长：speed(Hz/s) / 100 → Hz/ms
    float step_per_ms = (float)speed / 100.0f;

    // 4. 获取当前浮点频率和目标浮点频率
    float current_freq = smd_freq_gradient[ch].current_freq_float;
    float target_freq_float = (float)target_freq;

    // 5. 确定步长方向：升频（+）/降频（-）
    if (current_freq > target_freq_float)
    {
        step_per_ms = -step_per_ms; // 降频则步长为负
    }

    // 6. 剩余步长保护：避免单次步长超过目标值
    if ((step_per_ms > 0 && current_freq + step_per_ms > target_freq_float) ||
        (step_per_ms < 0 && current_freq + step_per_ms < target_freq_float))
    {
        step_per_ms = target_freq_float - current_freq; // 剩余步长直接到位
    }

    // 7. 设置渐变参数（全浮点型）
    smd_freq_gradient[ch].target_freq_float = target_freq_float; // 目标浮点频率
    smd_freq_gradient[ch].step_per_ms = step_per_ms;             // 浮点步长
    smd_freq_gradient[ch].is_running = 1;                        // 标记渐变开始

    return HAL_OK;
}

void SMD_PWM_StopFreqGradient(SMD_Channel ch)
{
    // 参数校验：避免数组越界
    if (ch >= SMD_CH_MAX)
    {
        return;
    }

    // 停止渐变：状态置0 + 步长置0（浮点型）
    smd_freq_gradient[ch].is_running = 0;
    smd_freq_gradient[ch].step_per_ms = 0.0f;
}

/* ========================= TIM10 1ms中断初始化（复用测试通过逻辑） ========================= */
HAL_StatusTypeDef SMD_TIM10_Init(void)
{
    // 1. 使能TIM10时钟（直接寄存器操作，避免HAL回调冲突）
    __HAL_RCC_TIM10_CLK_ENABLE();

    // 2. TIM10基础配置（84MHz系统时钟，1ms中断）
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 167;                  // 84MHz / (83999+1) = 1kHz
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;   // 向上计数
    htim10.Init.Period = 9999;                       // 0~999 → 1ms中断周期
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟不分频
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 使能ARR预装载

    // 初始化TIM10基础功能
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 3. 使能TIM10更新中断（核心：必须手动使能）
    __HAL_TIM_ENABLE_IT(&htim10, TIM_IT_UPDATE);

    // 4. NVIC配置（和测试代码一致，确保中断响应）
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0); // 主优先级0，子优先级0
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);         // 启用NVIC中断通道

    // 5. 启动TIM10（带中断）
    if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ========================= TIM10中断服务函数（整合到SMD.c） ========================= */
/**
  * @brief  TIM1更新/TIM10全局中断服务函数（1ms中断）
  * @note   全程浮点计算，全局变量保存浮点频率，仅更新硬件时转整型
  *         变频时自动保持 50% 占空比
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  // 检查TIM10更新中断标志
  if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) != RESET)
  {
	  num ++;
    // 先清中断标志
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);

    // 遍历所有通道
    for (int ch = 0; ch < SMD_CH_MAX; ch++)
    {
      // 只处理正在渐变的通道
      if (smd_freq_gradient[ch].is_running)
      {
        // ===================== 1. 浮点计算（全程浮点，不丢精度） =====================
        float current_freq = smd_freq_gradient[ch].current_freq_float;
        float target_freq  = smd_freq_gradient[ch].target_freq_float;
        float step         = smd_freq_gradient[ch].step_per_ms;

        // 新频率（浮点）
        float new_freq_float = current_freq + step;

        // 边界限制
        if (new_freq_float < SMD_PWM_FREQ_MIN)
          new_freq_float = SMD_PWM_FREQ_MIN;
        if (new_freq_float > SMD_PWM_FREQ_MAX)
          new_freq_float = SMD_PWM_FREQ_MAX;

        // 到达目标，停止渐变
        if (fabs(new_freq_float - target_freq) < 0.001f)
        {
          new_freq_float = target_freq;
          smd_freq_gradient[ch].is_running = 0;
        }

        // ===================== 2. 变化足够大才更新硬件 =====================
        if (fabs(new_freq_float - current_freq) >= 0.1f)
        {
          // 浮点转整型（给硬件用）
          uint32_t new_freq_int = (uint32_t)round(new_freq_float);

          

          // 计算新的 PSC 和 ARR
          uint32_t psc, arr;
          SMD_Calc_PSC_ARR(new_freq_int, &psc, &arr ,ch);
          
          // 重新配置定时器频率
          HAL_StatusTypeDef ret = HAL_ERROR;
		  // 先停PWM
          SMD_PWM_Stop((SMD_Channel)ch);
          switch(ch)
          {
            case SMD_CH0:
              htim2_smd.Init.Prescaler = psc;
              htim2_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim2_smd);
              break;
            case SMD_CH1:
              htim9_smd.Init.Prescaler = psc;
              htim9_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim9_smd);
              break;
            case SMD_CH2:
              htim5_smd.Init.Prescaler = psc;
              htim5_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim5_smd);
              break;
            case SMD_CH3:
              htim8_smd.Init.Prescaler = psc;
              htim8_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim8_smd);
              break;
            case SMD_CH4:
              htim13_smd.Init.Prescaler = psc;
              htim13_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim13_smd);
              break;
            case SMD_CH5:
              htim14_smd.Init.Prescaler = psc;
              htim14_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim14_smd);
              break;
            case SMD_CH6:
              htim1_smd.Init.Prescaler = psc;
              htim1_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim1_smd);
              break;
            case SMD_CH7:
              htim3_smd.Init.Prescaler = psc;
              htim3_smd.Init.Period    = arr;
              ret = HAL_TIM_PWM_Init(&htim3_smd);
              break;
            default:
              break;
          }

          if(ret == HAL_OK)
          {
            // ===================== 关键：变频后强制保持 50% 占空比 =====================
            uint32_t ccr_val = arr / 2;

            switch(ch)
            {
              case SMD_CH0: __HAL_TIM_SET_COMPARE(&htim2_smd,  TIM_CHANNEL_2, ccr_val); break;
              case SMD_CH1: __HAL_TIM_SET_COMPARE(&htim9_smd,  TIM_CHANNEL_1, ccr_val); break;
              case SMD_CH2: __HAL_TIM_SET_COMPARE(&htim5_smd,  TIM_CHANNEL_4, ccr_val); break;
              case SMD_CH3: __HAL_TIM_SET_COMPARE(&htim8_smd,  TIM_CHANNEL_1, ccr_val); break;
              case SMD_CH4: __HAL_TIM_SET_COMPARE(&htim13_smd, TIM_CHANNEL_1, ccr_val); break;
              case SMD_CH5: __HAL_TIM_SET_COMPARE(&htim14_smd, TIM_CHANNEL_1, ccr_val); break;
              case SMD_CH6: __HAL_TIM_SET_COMPARE(&htim1_smd,  TIM_CHANNEL_2, ccr_val); break;
              case SMD_CH7: __HAL_TIM_SET_COMPARE(&htim3_smd,  TIM_CHANNEL_4, ccr_val); break;
            }

            // 保存全局浮点频率（给下一次计算用）
            smd_freq_gradient[ch].current_freq_float = new_freq_float;
            smd_freq_gradient[ch].current_freq_int   = new_freq_int;

            // 重新开PWM
            SMD_PWM_Start((SMD_Channel)ch);
          }
        }
      }
    }
  }
}
