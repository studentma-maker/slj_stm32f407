#ifndef _SMD_H
#define _SMD_H

#include "main.h"
#include "gpio.h"
#include <stdint.h>

/* ========================= 参数宏定义 ========================= */

/**
 * @brief PWM频率范围
 * @note  SMD_PWM_FREQ_MIN 必须为 1，不可改为 0，同时具有两层含义：
 *
 *  【硬件约束】SMD_Calc_PSC_ARR() 和 SMD_PWM_SetDuty() 内部有
 *        "SMD_COUNTER_FREQ / freq" 和 "SMD_COUNTER_FREQ / current_freq_int"
 *        两处除法，freq=0 会导致除零硬件异常（HardFault）。
 *
 *  【停止语义】当目标频率 v_n 被设为 SMD_PWM_FREQ_MIN（1Hz）时，
 *        中断在 v_c 减速到达 1Hz 后会自动调用 SMD_PWM_Stop() 并置
 *        is_running=0，电机真正停止输出，不会出现"1步/秒蠕动"现象。
 *        因此，"S曲线减速停止"的正确用法是将目标频率写为 1（即 PU=1），
 *        而不是写为 0。直接急停请用 SP=0。
 */
#define SMD_PWM_FREQ_MIN        1u      // 最低频率 Hz（禁止改为0，见上方说明）
#define SMD_PWM_FREQ_MAX        8000u   // 最高频率 Hz
#define SMD_COUNTER_FREQ        1000000u // 计数器频率 1MHz

/**
 * @brief S曲线加速度（ACC）参数范围
 * @note  ACC 含义为"加速度的上限值"（Hz/s）。
 *        S曲线过程：a_c 以 JERK 速率从 0 增大到 ACC_MAX，
 *        到达 ACC_MAX 后进入匀加速段，接近目标时再对称减速。
 *        单位 Hz/s，通过 Modbus SMD_ACC_DATA[] 读写。
 */
#define SMD_ACC_MAX_MIN         1u      // 最大加速度下限 Hz/s（防止为0导致永远不动）
#define SMD_ACC_MAX_MAX         4000u  // 最大加速度上限 Hz/s（可根据电机实际调整）
#define SMD_ACC_MAX_DEFAULT			1000u
/**
 * @brief S曲线 Jerk 参数范围
 * @note  Jerk 含义为"加速度的变化率"（Hz/s²），决定S曲线的"柔和程度"。
 *        Jerk 越大，加速度增长越快，S曲线越陡；Jerk 越小，加速越平滑。
 *        单位 Hz/s²，通过 Modbus SMD_JERK_DATA[] 读写。
 */
#define SMD_JERK_MIN            1u      // Jerk下限 Hz/s²（防止为0导致除零）
#define SMD_JERK_MAX            4000u 	// Jerk上限 Hz/s²（可根据实际调整）
#define SMD_JERK_DEFAULT        2000u   // 默认Jerk Hz/s²（上电初始值）

/* TIM10 中断周期 */
#define SMD_UPDATE_DT_s           0.001f  // 1ms = 0.001s
#define SMD_UPDATE_DT_ms          1u  	  // 1ms

/* 兼容旧接口的渐变参数范围（SMD_PWM_SetFreqGradient speed参数） */
#define SMD_GRADIENT_MIN        SMD_ACC_MAX_MIN
#define SMD_GRADIENT_MAX        SMD_ACC_MAX_MAX

/* ========================= 通道枚举 ========================= */
typedef enum
{
    SMD_CH0 = 0,  // PA1 - TIM2_CH2
    MOTOR_PaiFa = SMD_CH0 ,//排发电机别名
    SMD_CH1,      // PA2 - TIM9_CH1
    MOTOR_UpDown = SMD_CH1 ,//升降电机别名
    SMD_CH2,      // PA3 - TIM5_CH4
    MOTOR_FBack = SMD_CH2 ,//进退电机别名
    SMD_CH3,      // PA5 - TIM8_CH1N
    MOTOR_Trans = SMD_CH3 ,//送发电机别名
    SMD_CH4,      // PA6 - TIM13_CH1
    MOTOR_GripperMove = SMD_CH4,
    SMD_CH5,      // PA7 - TIM14_CH1
    MOTOR_Feed = SMD_CH5 ,//上料电机别名
    SMD_CH6,      // PB0 - TIM1_CH2N
    SMD_CH7,      // PB1 - TIM3_CH4
    SMD_CH_MAX
} SMD_Channel;

/* ========================= 方向切换状态 ========================= */
typedef enum
{
    SMD_DIR_NORMAL    = 0,  // 正常运行
    SMD_DIR_DECEL     = 1,  // 正在减速到0（换向中）
    SMD_DIR_WAIT      = 2,  // 已到0，等待GPIO切换完成
} SMD_DirState;

/* ========================= S曲线渐变参数结构体 ========================= */
/**
 * @brief 每路PWM通道的S曲线运动状态
 * @note  移植自ESP32 motor_smctl_t，全部使用浮点计算
 *        v_c: 当前速度（Hz，浮点），对应硬件输出频率
 *        a_c: 当前加速度（Hz/s）
 *        v_n: 目标速度（Hz，浮点）
 *        jerk: 当前jerk（Hz/s²）
 */
typedef struct
{
    /* --- S曲线核心变量（单位：Hz 和 Hz/s） --- */
    float    v_c;               // 当前速度（浮点）
    float    a_c;               // 当前加速度
    float    v_n;               // 目标速度
    float    jerk;              // 当前jerk值

    /* --- 方向切换 --- */
    uint8_t  dir_change;        // 1=需要换向
    SMD_DirState dir_state;     // 换向子状态机

    /* --- 硬件同步 --- */
    uint32_t current_freq_int;  // 当前整型频率（给CCR计算用）
    uint8_t  is_running;        // 渐变是否激活（1=激活）

} SMD_Freq_Gradient;

/* ========================= 外部变量声明 ========================= */
extern SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX];
extern TIM_HandleTypeDef htim10;

/* 电机参数数组，供Modbus读写 */
extern uint16_t SMD_PU_DATA[8];   // 目标频率 Hz
extern uint16_t SMD_ACC_DATA[8];  // 最大加速度 Hz/s
extern uint16_t SMD_JERK_DATA[8]; // Jerk Hz/s²，S曲线加加速度，用户可通过Modbus自定义
extern uint16_t GripperCurStepsU;   // 夹爪当前步数整数部分
typedef struct
{
    uint8_t  is_running;
    uint16_t targetSteps;
} GripperStepsCtl_t;
extern GripperStepsCtl_t g_gripperStepsCtl;   // 夹爪当前步数整数部分
/* ========================= 函数声明 ========================= */

/* 初始化 */
HAL_StatusTypeDef SMD_PWM_Init(SMD_Channel ch);
HAL_StatusTypeDef SMD_PWM_InitAll(void);
HAL_StatusTypeDef SMD_TIM10_Init(void);

/* 启停 */
HAL_StatusTypeDef SMD_PWM_Start(SMD_Channel ch);
HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch);
HAL_StatusTypeDef SMD_PWM_StartAll(void);
HAL_StatusTypeDef SMD_PWM_StopAll(void);

/* 占空比 */
void SMD_PWM_SetDuty(SMD_Channel ch, uint32_t duty);
void SMD_PWM_SetDutyPercent(SMD_Channel ch, float percent);

/* 频率直接设置 */
HAL_StatusTypeDef SMD_PWM_SetFreq(SMD_Channel ch, uint32_t freq);

/* S曲线渐变接口（对外，兼容旧Modbus接口） */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t accel);
void              SMD_PWM_StopFreqGradient(SMD_Channel ch);

#endif /* _SMD_H */
