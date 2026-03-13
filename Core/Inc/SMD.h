#ifndef __SMD_H
#define __SMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ========================= 核心配置宏定义（带详细说明） ========================= */
/**
 * @brief PWM输出频率范围限定
 * @note  硬件/应用层约束，禁止超出此范围配置
 */
#define SMD_PWM_FREQ_MIN    1u          // PWM输出频率下限（1Hz）
#define SMD_PWM_FREQ_MAX    4000u       // PWM输出频率上限（4000Hz）

/**
 * @brief 频率渐变速度范围限定
 * @note  渐变速度单位为Hz/s，禁止超出此范围配置
 */
#define SMD_GRADIENT_MIN    1u          // 渐变速度下限（1Hz/s）
#define SMD_GRADIENT_MAX    1000u       // 渐变速度上限（1000Hz/s）

/**
 * @brief PWM计数器基准频率
 * @note  固定为1MHz（1us分辨率），便于占空比和频率计算
 */
#define SMD_COUNTER_FREQ    50000u    

/* ========================= 频率渐变控制结构体（核心重构） ========================= */
/**
 * @brief 单个PWM通道的频率渐变控制结构体
 * @note  核心设计：
 *        1. 所有计算/判断使用浮点型变量，仅在更新PWM硬件时转换为整型
 *        2. 浮点型变量保存实时值，保证渐变精度不丢失
 *        3. 整型变量仅用于硬件配置，不参与渐变计算
 */
typedef struct {
    float target_freq_float;    // 目标频率（浮点型，1.0~4000.0Hz）
    float current_freq_float;   // 当前频率（浮点型，核心：保存实时值用于下次计算）
    float step_per_ms;          // 每ms频率变化步长（浮点型，Hz/ms）
    uint8_t is_running;         // 渐变状态标记 (0:停止, 1:运行中)
    uint32_t current_freq_int;  // 当前频率（整型，仅用于PWM硬件配置）
} SMD_Freq_Gradient;

/* ========================= PWM通道枚举定义（带硬件映射） ========================= */
/**
 * @brief 8路PWM通道枚举
 * @note  每个通道对应固定的GPIO口和定时器通道，不可随意修改
 */
typedef enum {
    SMD_CH0 = 0,  // PA1 - TIM2_CH2 (普通PWM通道)
    SMD_CH1 = 1,  // PA2 - TIM9_CH1 (普通PWM通道)
    SMD_CH2 = 2,  // PA3 - TIM5_CH4 (普通PWM通道)
    SMD_CH3 = 3,  // PA5 - TIM8_CH1N (高级定时器互补通道)
    SMD_CH4 = 4,  // PA6 - TIM13_CH1 (普通PWM通道)
    SMD_CH5 = 5,  // PA7 - TIM14_CH1 (普通PWM通道)
    SMD_CH6 = 6,  // PB0 - TIM1_CH2N (高级定时器互补通道)
    SMD_CH7 = 7,  // PB1 - TIM3_CH4 (普通PWM通道)
    SMD_CH_MAX    // 通道总数（仅用于遍历，不可作为实际通道使用）
} SMD_Channel;

/* ========================= 全局定时器句柄声明 ========================= */
// 普通PWM通道定时器句柄
extern TIM_HandleTypeDef htim2_smd;
extern TIM_HandleTypeDef htim9_smd;
extern TIM_HandleTypeDef htim5_smd;
extern TIM_HandleTypeDef htim13_smd;
extern TIM_HandleTypeDef htim14_smd;
extern TIM_HandleTypeDef htim3_smd;

// 高级定时器句柄（互补通道）
extern TIM_HandleTypeDef htim8_smd;
extern TIM_HandleTypeDef htim1_smd;

// TIM10定时器句柄（1ms中断，频率渐变驱动核心）
extern TIM_HandleTypeDef htim10;

extern uint16_t SMD_PU_DATA[8];
extern uint16_t SMD_ACC_DATA[8];
/* ========================= 全局渐变参数声明 ========================= */
/**
 * @brief 8路通道的渐变参数数组
 * @note  全局可见，中断服务函数直接操作此数组完成渐变计算
 */
extern SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX];

/* ========================= 函数声明（带功能+参数+返回值说明） ========================= */

/**
 * @brief  初始化指定通道的PWM功能
 * @param  ch: 要初始化的通道（SMD_Channel枚举值）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=失败
 * @note   1. 初始化后通道默认频率为200Hz，占空比0%
 *         2. 自动完成GPIO和定时器的底层配置
 */
HAL_StatusTypeDef SMD_PWM_Init(SMD_Channel ch);

/**
 * @brief  初始化所有8路PWM通道 + TIM10 1ms中断
 * @retval HAL_StatusTypeDef: HAL_OK=全部成功，HAL_ERROR=至少一个通道失败
 * @note   上电后优先调用此函数完成全局初始化
 */
HAL_StatusTypeDef SMD_PWM_InitAll(void);

/**
 * @brief  启动指定通道的PWM输出
 * @param  ch: 要启动的通道（SMD_Channel枚举值）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=失败
 * @note   高级定时器互补通道（CH3/CH6）自动调用HAL_TIMEx_PWMN_Start
 */
HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch);

/**
 * @brief  停止指定通道的PWM输出
 * @param  ch: 要停止的通道（SMD_Channel枚举值）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=失败
 */
HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch);

/**
 * @brief  启动所有8路PWM通道输出
 * @retval HAL_StatusTypeDef: HAL_OK=全部成功，HAL_ERROR=至少一个通道失败
 */
HAL_StatusTypeDef SMD_PWM_StartAll(void);

/**
 * @brief  停止所有8路PWM通道输出
 * @retval HAL_StatusTypeDef: HAL_OK=全部成功，HAL_ERROR=至少一个通道失败
 */
HAL_StatusTypeDef SMD_PWM_StopAll(void);

/**
 * @brief  设置指定通道的PWM占空比（计数值模式）
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @param  duty: 占空比计数值（0 ~ 计数器最大值）
 * @note   1. 计数器最大值 = SMD_COUNTER_FREQ / current_freq_int
 *         2. 函数内自动做边界保护，超出最大值则设为最大值
 */
void SMD_PWM_SetDuty(SMD_Channel ch, uint32_t duty);

/**
 * @brief  设置指定通道的PWM占空比（百分比模式）
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @param  percent: 占空比百分比（0.0f ~ 100.0f）
 * @note   1. 自动将百分比转换为计数值，浮点运算保证精度
 *         2. 超出范围自动修正为0或100
 */
void SMD_PWM_SetDutyPercent(SMD_Channel ch, float percent);

/**
 * @brief  直接设置指定通道的PWM输出频率（立即生效）
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @param  freq: 目标频率（1~4000Hz，超出范围返回错误）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=参数非法/配置失败
 * @note   1. 设置时先停止PWM输出，配置完成后重启
 *         2. 同步更新current_freq_float和current_freq_int
 */
HAL_StatusTypeDef SMD_PWM_SetFreq(SMD_Channel ch, uint32_t freq);

/**
 * @brief  设置指定通道的频率渐变（平滑变到目标频率）
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @param  target_freq: 目标频率（1~4000Hz）
 * @param  speed: 渐变速度（1~1000Hz/s）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=参数非法
 * @note   1. 速度单位为Hz/s，函数内转换为Hz/ms（浮点型）
 *         2. 自动判断升/降频方向，设置step_per_ms正负
 *         3. 启动渐变前自动停止当前渐变，避免叠加
 */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t speed);

/**
 * @brief  停止指定通道的频率渐变
 * @param  ch: 目标通道（SMD_Channel枚举值）
 * @note   1. 停止后current_freq_float保留当前浮点值
 *         2. is_running置0，step_per_ms置0
 */
void SMD_PWM_StopFreqGradient(SMD_Channel ch);

/**
 * @brief  初始化TIM10定时器（1ms中断，驱动频率渐变）
 * @retval HAL_StatusTypeDef: HAL_OK=成功，HAL_ERROR=失败
 * @note   1. 复用测试验证过的初始化逻辑，确保100%进中断
 *         2. 直接寄存器配置，避免HAL回调冲突
 */
HAL_StatusTypeDef SMD_TIM10_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SMD_H */
