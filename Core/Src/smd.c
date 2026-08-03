#include "SMD.h"
#include "main.h"
#include "mb_hook.h"
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
uint16_t MotorCurStepsU[SMD_CH_MAX] = {0};
static uint32_t MotorCurStepsSub[SMD_CH_MAX] = {0};  /* 子步累加器 (1步=1000子步)，消除浮点累积误差 */
#define GripperToOriginPU   1600
#define UpDownToOriginPU    1600

typedef enum  {
    MOTOR_GripperMoveDR_F = 0,  //夹爪往排发方向运行:feed
    MOTOR_GripperMoveDR_G,      //夹爪夹取假发:grip
    MOTOR_GripperMoveDR_t_MAX
}MOTOR_GripperMoveDR_t;
GrippertoOrigin_P g_sysToOrigin = defaultset;
GripperStepsCtl_t g_motorStepsCtl[SMD_CH_MAX] = {
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
    {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0},
};

/**
 * @brief  8路PWM通道的S曲线运动状态
 * @note   v_c/v_n 单位 Hz（浮点），a_c 单位 Hz/s，jerk 字段为运行时动态值
 *         acc_max / jerk 的取值规则：
 *         - 步数控制（g_motorStepsCtl[ch].is_running）：使用 SMD_ACC_MAX_DEFAULT / SMD_JERK_DEFAULT
 *         - 指令控制（Modbus 写 PU）：使用 SMD_ACC_DATA[ch] / SMD_JERK_DATA[ch]（用户可自定义）
 */
SMD_Freq_Gradient smd_freq_gradient[SMD_CH_MAX] = {
    /* v_c   a_c   v_n   jerk  acc_max dir_change dir_state  freq_int  is_running */
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH0
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH1
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH2
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH3
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH4
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH5
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH6
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, SMD_DIR_NORMAL, 1, 0},  // CH7
};

/* ========================= 通道 -> 定时器映射表 =========================
 * 把"哪个通道用哪个定时器/哪个比较通道/是否为互补输出"集中到一张表里，
 * 避免 Init/Start/Stop/SetDuty/SetFreq/ApplyFreqToHW 里各写一遍8路的
 * switch-case（原逻辑完全不变，只是不再重复7次）。
 * is_complementary=1 的通道（CH3=TIM8, CH6=TIM1）需要用 HAL_TIMEx_PWMN_xxx，
 * 其余通道用普通的 HAL_TIM_PWM_xxx。
 */
typedef struct
{
    TIM_HandleTypeDef *handle;           // 对应的 htimX_smd 句柄地址
    TIM_TypeDef        *instance;        // 对应的 TIMx 外设实例
    uint32_t            tim_channel;     // TIM_CHANNEL_x
    uint8_t             is_complementary;// 1=需要用 PWMN（互补通道）接口
} SMD_TimerMap;

static const SMD_TimerMap smd_timer_map[SMD_CH_MAX] =
{
    /* handle          instance  channel         is_complementary */
    { &htim2_smd,  TIM2,  TIM_CHANNEL_2, 0 },  // CH0 - PA1
    { &htim9_smd,  TIM9,  TIM_CHANNEL_1, 0 },  // CH1 - PA2
    { &htim5_smd,  TIM5,  TIM_CHANNEL_4, 0 },  // CH2 - PA3
    { &htim8_smd,  TIM8,  TIM_CHANNEL_1, 1 },  // CH3 - PA5 (TIM8_CH1N)
    { &htim13_smd, TIM13, TIM_CHANNEL_1, 0 },  // CH4 - PA6
    { &htim14_smd, TIM14, TIM_CHANNEL_1, 0 },  // CH5 - PA7
    { &htim1_smd,  TIM1,  TIM_CHANNEL_2, 1 },  // CH6 - PB0 (TIM1_CH2N)
    { &htim3_smd,  TIM3,  TIM_CHANNEL_4, 0 },  // CH7 - PB1
};

/* ========================= 内部函数前置声明 ========================= */
static HAL_StatusTypeDef SMD_Init_HighTimer  (TIM_HandleTypeDef *htim, uint32_t psc, uint32_t arr, uint32_t tim_channel);
static HAL_StatusTypeDef SMD_Init_NormalTimer(TIM_HandleTypeDef *htim, uint32_t psc, uint32_t arr, uint32_t tim_channel);
static void     SMD_Calc_PSC_ARR    (uint32_t freq, uint32_t *psc, uint32_t *arr, int ch);
static void     SMD_AccumulateSteps (SMD_Channel ch, uint32_t freq_int);
static void     SMD_ApplyFreqToHW   (SMD_Channel ch, uint32_t freq_int);
static void     SMD_UpdateVelocity  (SMD_Freq_Gradient *m, float delta_v, float accel_max, float jerk);
static uint8_t  SMD_IsLimited       (int ch, uint8_t cur_dir, SMD_Freq_Gradient *m);
static uint16_t MotorStepsMaxPU     (SMD_Channel ch, uint8_t dir);
static void     SMD_MotorStepsCtl   (SMD_Channel ch);
static void     SMD_SysToOrigin     (void);
static void     SMD_ProcessChannel  (SMD_Channel ch);
static void     SMD_RunSCurve       (SMD_Channel ch, SMD_Freq_Gradient *m);
static void     SMD_CheckRelayMotorLimit(void);

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
static void SMD_AccumulateSteps(SMD_Channel ch, uint32_t freq_int)
{
    MotorCurStepsSub[ch] += freq_int;  /* freq_int 子步/ms，1000子步 = 1整步 */
    uint16_t whole = (uint16_t)(MotorCurStepsSub[ch] / 1000u);
    uint8_t dir = SMD_DR_READ(ch);
    if(ch == MOTOR_FBack) dir = !dir;
    if (whole > 0)
    {
        MotorCurStepsSub[ch] -= (uint32_t)whole * 1000u;
        if (dir)
            MotorCurStepsU[ch] += whole;
        else {
            if (MotorCurStepsU[ch] < whole) MotorCurStepsU[ch] = 0;
            else MotorCurStepsU[ch] -= whole;
        }
    }
}

/**
 * @brief  仅更新PSC/ARR寄存器，不停止PWM，适合中断内调用
 * @note   利用 ARR 预装载，修改在下一个周期生效，无毛刺
 */
static void SMD_ApplyFreqToHW(SMD_Channel ch, uint32_t freq_int)
{
    if (ch >= SMD_CH_MAX) return;

    uint32_t psc, arr;
    SMD_Calc_PSC_ARR(freq_int, &psc, &arr, (int)ch);

    /* 直接写寄存器，避免在中断内调用完整HAL_TIM_PWM_Init带来的开销。
     * 注意：不主动置位 EGR(UG) 强制更新，PSC/ARR/CCR 都走影子寄存器，
     * 在当前周期结束时自动生效，不会产生毛刺。 */
    const SMD_TimerMap *t = &smd_timer_map[ch];
    t->handle->Instance->PSC = psc;
    t->handle->Instance->ARR = arr;
    __HAL_TIM_SET_COMPARE(t->handle, t->tim_channel, arr / 2);  // 保持50%占空比

    smd_freq_gradient[ch].current_freq_int = freq_int;
    if (ch == MOTOR_GripperMove || ch == MOTOR_UpDown || ch == MOTOR_FBack)
        SMD_AccumulateSteps(ch, freq_int);
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

    const SMD_TimerMap *t = &smd_timer_map[ch];
    t->handle->Instance = t->instance;

    uint32_t psc, arr;
    /* 初始化时以1Hz计算PSC，后续由中断动态调频 */
    SMD_Calc_PSC_ARR(1, &psc, &arr, (int)ch);

    return t->is_complementary
               ? SMD_Init_HighTimer  (t->handle, psc, arr, t->tim_channel)
               : SMD_Init_NormalTimer(t->handle, psc, arr, t->tim_channel);
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
    if (ch >= SMD_CH_MAX) return HAL_ERROR;
    const SMD_TimerMap *t = &smd_timer_map[ch];
    return t->is_complementary ? HAL_TIMEx_PWMN_Start(t->handle, t->tim_channel)
                                : HAL_TIM_PWM_Start   (t->handle, t->tim_channel);
}

HAL_StatusTypeDef SMD_PWM_Stop(SMD_Channel ch)
{
    if (ch >= SMD_CH_MAX) return HAL_ERROR;
    const SMD_TimerMap *t = &smd_timer_map[ch];
    return t->is_complementary ? HAL_TIMEx_PWMN_Stop(t->handle, t->tim_channel)
                                : HAL_TIM_PWM_Stop   (t->handle, t->tim_channel);
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

    const SMD_TimerMap *t = &smd_timer_map[ch];
    __HAL_TIM_SET_COMPARE(t->handle, t->tim_channel, duty);
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

    const SMD_TimerMap *t = &smd_timer_map[ch];
    t->handle->Init.Prescaler = psc;
    t->handle->Init.Period    = arr;
    ret = HAL_TIM_PWM_Init(t->handle);
    if (ret != HAL_OK) return ret;

    smd_freq_gradient[ch].current_freq_int = freq;

    return SMD_PWM_Start(ch);
}

/* ========================= S曲线渐变接口（对外） ========================= */
/**
 * @brief  设置目标频率和最大加速度，启动S曲线渐变
 * @param  ch:          通道
 * @param  target_freq: 目标频率（Hz），范围 SMD_PWM_FREQ_MIN ~ SMD_PWM_FREQ_MAX
 * @param  accel:       最大加速度 ACC_MAX（Hz/s），范围 SMD_ACC_MAX_MIN ~ SMD_ACC_MAX_MAX
 *                      存入 m->acc_max，不覆盖 SMD_ACC_DATA[ch]。
 *                      S曲线加速过程：a_c 以 jerk 从0增大到 accel，
 *                      到达 accel 后匀加速，接近目标时对称减速至0。
 */
HAL_StatusTypeDef SMD_PWM_SetFreqGradient(SMD_Channel ch, uint32_t target_freq, uint32_t accel)
{
    if (ch >= SMD_CH_MAX)                                              return HAL_ERROR;
    if (target_freq < SMD_PWM_FREQ_MIN || target_freq > SMD_PWM_FREQ_MAX) return HAL_ERROR;
    if (accel < SMD_ACC_MAX_MIN || accel > SMD_ACC_MAX_MAX)            return HAL_ERROR;

    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];

    m->v_n        = (float)target_freq;
    m->acc_max    = (float)accel;  // 本次运动的加速度上限，不影响用户写入的 SMD_ACC_DATA

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
int32_t SMD_CalcAccNeedSteps(float v_current, float a_current,
                              float v_target,  float acc_max, float jerk)
{
    if (v_current <= v_target) return 0;
    if (jerk <= 0.0f) return 0;

    double vc = (double)v_current;
    double ac = (double)a_current;
    double vt = (double)v_target;
    double j  = (double)jerk;
    double am = (double)acc_max;

    double total = 0.0;

    /* ---------------------------------------------------------------
     * 阶段0：若 a_c < acc_max，制动后 a_c 会先继续增大到 acc_max
     *         同时速度已经开始减小（制动方向）
     *   t0       = (am - ac) / j
     *   s0       = vc*t0 - 0.5*ac*t0² - (j/6)*t0³
     *   v_after0 = vc - ac*t0 - 0.5*j*t0²
     * --------------------------------------------------------------- */
    double vc_A = vc;   // 进入段A时的速度
    if (ac < am)
    {
        double t0      = (am - ac) / j;
        double s0      = vc * t0 - 0.5 * ac * t0 * t0 - (j / 6.0) * t0 * t0 * t0;
        double v_after = vc - ac * t0 - 0.5 * j * t0 * t0;
        if (s0 < 0.0) s0 = 0.0;
        if (v_after <= vt) return (int32_t)(s0 + 0.5);
        total += s0;
        vc_A   = v_after;
    }
    /* ac == am 时直接进入段A，vc_A = vc */

    /* ---------------------------------------------------------------
     * 段A：以 acc_max 匀减速，直到剩余速差 = decel_dist = 0.5*am²/j
     *   vc_mid = vt + 0.5*am²/j
     *   s_A    = (vc_A² - vc_mid²) / (2*am)
     * --------------------------------------------------------------- */
    double vc_mid = vt + 0.5 * am * am / j;
    if (vc_A > vc_mid)
    {
        double s_A = (vc_A * vc_A - vc_mid * vc_mid) / (2.0 * am);
        total += s_A;
    }
    else
    {
        vc_mid = vc_A;  // 已在S曲线收尾范围内，跳过段A
    }

    /* ---------------------------------------------------------------
     * 段B：S曲线收尾，a_c 从 am 线性降到 0，速度从 vc_mid 减至 vt
     *   t_B = am / j
     *   s_B = vc_mid*t_B - 0.5*am*t_B² + (j/6)*t_B³
     * --------------------------------------------------------------- */
    double t_B = am / j;
    double s_B = vc_mid * t_B - 0.5 * am * t_B * t_B + (j / 6.0) * t_B * t_B * t_B;
    total += s_B;

    return (int32_t)(total + 0.5);
}
static uint8_t SMD_IsLimited(int ch,uint8_t cur_dir,SMD_Freq_Gradient *m)
{
    uint8_t hit = 0;
    switch (ch)
    {
      case MOTOR_UpDown:
          hit = (!IN_READ(0) && !cur_dir);
          break;
      case MOTOR_FBack:
          hit = ((!IN_READ(1) && !cur_dir) || (!IN_READ(2) && cur_dir));
          break;
      case MOTOR_GripperMove:
          hit = ((!IN_READ(5) && !cur_dir) || 
		  		 (!IN_READ(6) && cur_dir) ||
		  		 (IN_READ(3)) ||
		  		 (!OUT_READ(RELAY_1)));
          break;
      default:
          break;
    }
    if(!IN_READ(19)) hit = 1;//急停按钮
    if(hit)
    {
      SMD_PWM_Stop((SMD_Channel)ch);
      SMD_PU_DATA[ch]     = SMD_PWM_FREQ_MIN;
      m->v_c              = 0.0f;
      m->a_c              = 0.0f;
      m->v_n              = (float)SMD_PWM_FREQ_MIN;
      m->is_running       = 0;
      m->current_freq_int = SMD_PWM_FREQ_MIN;
      m->dir_change       = 0;
      m->dir_state        = SMD_DIR_NORMAL;
      if (g_motorStepsCtl[ch].is_running)
      {
          g_motorStepsCtl[ch].is_running = 0;
          g_motorStepsCtl[ch].braking = 0;
      }
      /* 触发原点限位时步数清零 */
      if (ch == MOTOR_GripperMove && !IN_READ(5))
      {
          MotorCurStepsU[ch] = 0;
          MotorCurStepsSub[ch] = 0;
      }
    }
    return hit;
}

/**
 * @brief  步数控制的目标运行速度（Hz），按通道 + 运动方向查表
 * @param  ch:  电机通道
 * @param  dir: 该次运动对应的 DR 电平（0 或 1，与 SMD_DR_READ() 同一约定）
 * @note   夹爪电机（MOTOR_GripperMove）区分方向：
 *           dir=0：夹持物料运动，负载较大 → GRIPPER_MOVE_MAXPU（原速度2000Hz，不变）
 *           dir=1：空载返回运动，无负载   → GRIPPER_MOVE_MAXPU_DIR1（更快，3000Hz）
 *         其余通道（升降、进退）两个方向暂共用同一速度。
 *         如果后续需要给其他通道也做方向区分，仿照 MOTOR_GripperMove 的写法
 *         加一个 case、在 SMD.h 里加一对 _DIR0/_DIR1 宏即可。
 */
static uint16_t MotorStepsMaxPU(SMD_Channel ch, uint8_t dir)
{
    switch (ch) {
        case MOTOR_GripperMove: return dir ? GRIPPER_MOVE_MAXPU_DIR1 : GRIPPER_MOVE_MAXPU;
        case MOTOR_UpDown:      return UPDOWN_MOVE_MAXPU;
        case MOTOR_FBack:       return FBACK_MOVE_MAXPU;
        default: return 0;
    }
}
static void SMD_MotorStepsCtl(SMD_Channel ch)
{
    if (!g_motorStepsCtl[ch].is_running) return;

    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];
    int32_t step_remain = (int32_t)g_motorStepsCtl[ch].targetSteps - (int32_t)MotorCurStepsU[ch];

    /* ============================================================
     * 限位直达模式：targetSteps==0 向原点限位，targetSteps==0xFFFF 向远端限位
     * 升降电机  cur_dir=0 上升→上限位 IN_READ(0)，只有原点限位，不支持 0xFFFF
     * 夹爪电机  cur_dir=0→IN_READ(5)  cur_dir=1→IN_READ(6)
     * 进退电机  targetSteps==0 走正常步数控制回零，不走限位直达
     * ============================================================ */
    if ((g_motorStepsCtl[ch].targetSteps == 0 && ch != MOTOR_FBack) ||
        (g_motorStepsCtl[ch].targetSteps == 0xFFFFu && ch != MOTOR_UpDown))
    {
        uint8_t target_dir = (g_motorStepsCtl[ch].targetSteps == 0) ? 0u : 1u;
        if (ch == MOTOR_FBack) target_dir = 0u;  // FBack: DR=0时步数增加，远端限位也走0方向
        uint16_t max_pu = MotorStepsMaxPU(ch, target_dir);  // 按目标方向取速度（夹爪两方向不同）

        if (SMD_DR_READ(ch) != target_dir || SMD_PU_DATA[ch] != max_pu || !m->is_running || m->dir_change)
        {
            SMD_PWM_Stop(ch);
            SMD_DR(ch, target_dir);
            m->v_c = 0.0f;
            m->a_c = 0.0f;
            m->dir_change = 0;
            m->dir_state = SMD_DIR_NORMAL;

            SMD_PU_DATA[ch] = max_pu;
            SMD_PWM_SetFreqGradient(ch, max_pu, SMD_ACC_MAX_DEFAULT);
        }
        return;
    }

    /* ============================================================
     * 阶段1：S曲线运动 (is_running == 1)
     * ============================================================ */

    /* --- 1a. 刹车监视：每 tick 检查 step_remain，归零立刻停 --- */
    if (g_motorStepsCtl[ch].braking)
    {
        if (step_remain == 0)
        {
            SMD_PWM_Stop(ch);
            m->v_c              = 0.0f;
            m->a_c              = 0.0f;
            m->is_running       = 0;
            m->current_freq_int = SMD_PWM_FREQ_MIN;
            SMD_PU_DATA[ch]     = SMD_PWM_FREQ_MIN;
            g_motorStepsCtl[ch].is_running = 0;
            g_motorStepsCtl[ch].braking    = 0;
        }
        return;
    }

    /* --- 1b. 正常运动决策 --- */

    /* 已精确到达目标：立即停止 */
    if (step_remain == 0)
    {
        SMD_PWM_Stop(ch);
        m->v_c              = 0.0f;
        m->a_c              = 0.0f;
        m->is_running       = 0;
        m->current_freq_int = SMD_PWM_FREQ_MIN;
        SMD_PU_DATA[ch] = SMD_PWM_FREQ_MIN;
        g_motorStepsCtl[ch].is_running = 0;
        return;
    }

    uint8_t need_dir = (step_remain > 0) ? 1u : 0u;
    if (ch == MOTOR_FBack) need_dir = !need_dir;  // FBack方向反逻辑：DR=0时步数增加
    uint16_t max_pu = MotorStepsMaxPU(ch, need_dir);  // 按本次实际运行方向取速度（夹爪两方向不同）

    /* 换向处理 */
    if (!m->dir_change && m->dir_state == SMD_DIR_NORMAL)
    {
        if (need_dir != (uint8_t)SMD_DR_READ(ch))
        {
            m->is_running = 1;
            m->dir_change = 1;
            m->dir_state  = SMD_DIR_NORMAL;
            return;
        }
    }
    else
    {
        m->is_running = 1;
        return;
    }

    /* 方向正确，计算刹车距离：减速到 100Hz 所需步数 + 100 步巡航缓冲 */
    m->is_running = 1;
    int32_t abs_remain = (step_remain >= 0) ? step_remain : -step_remain;

    int32_t brake_steps = SMD_CalcAccNeedSteps(
                  m->v_c, m->a_c,
                  (float)BRAKE_TARGET_HZ,
                  (float)SMD_ACC_MAX_DEFAULT,
                  (float)SMD_JERK_DEFAULT);
    if (brake_steps < 0) brake_steps = 0;

    uint16_t freq_int;
    if (abs_remain < brake_steps + BRAKE_BUFFER)
    {
        freq_int = BRAKE_TARGET_HZ;
    }
    else
    {
        /* 预测下一 tick 剩余步数，若小于刹车距离则必须此刻刹车 */
        int32_t next_remain = abs_remain - (int32_t)(m->v_c / 1000u);
        if (next_remain < brake_steps)
            freq_int = BRAKE_TARGET_HZ;
        else
            freq_int = max_pu;
    }

    if (freq_int != SMD_PU_DATA[ch])
    {
        SMD_PU_DATA[ch] = freq_int;
        SMD_PWM_SetFreqGradient(ch, freq_int, SMD_ACC_MAX_DEFAULT);

        if (freq_int == BRAKE_TARGET_HZ)
            g_motorStepsCtl[ch].braking = 1;
    }
}
static void SMD_SysToOrigin(void)
{
    switch(g_sysToOrigin)
    {
          case defaultset:
              OUT(RELAY_1, 1);
              OUT(RELAY_FeedHair, RELAY_FeedHair_up);
              OUT(RELAY_PressHair, RELAY_PressHair_up);
              OUT(RELAY_WarnYELLOW, WarnLED_on);
              mbsUSB.regHoldingBuf[OUT_8_ADDR] = WarnLED_on;
              mbsUSB.regHoldingBuf[OUT_3_ADDR] = RELAY_PressHair_up;
              mbsUSB.regHoldingBuf[OUT_2_ADDR] = RELAY_FeedHair_up;
              mbsESP.regHoldingBuf[OUT_8_ADDR] = WarnLED_on;
              mbsESP.regHoldingBuf[OUT_3_ADDR] = RELAY_PressHair_up;
              mbsESP.regHoldingBuf[OUT_2_ADDR] = RELAY_FeedHair_up;
              g_sysToOrigin++;
              return;
          case gripperSetDR:
              if (MOTOR_GripperMoveDR_F != SMD_DR_READ(MOTOR_GripperMove))
              {
                  smd_freq_gradient[MOTOR_GripperMove].is_running = 1;
                  smd_freq_gradient[MOTOR_GripperMove].dir_change = 1;
                  smd_freq_gradient[MOTOR_GripperMove].dir_state  = SMD_DIR_NORMAL;
              }
              g_sysToOrigin++;
              return;
          case waitPressUP:
              if ((!IN_READ(3) && !IN_READ(4) && IN_READ(19)/*急停*/) || !IN_READ(5))
              {
                  g_sysToOrigin++;
              }
              return;
          case gripperSettoOriginPU:
              if (!smd_freq_gradient[MOTOR_GripperMove].dir_change &&
                   smd_freq_gradient[MOTOR_GripperMove].dir_state == SMD_DIR_NORMAL)
              {
                  if (GripperToOriginPU != SMD_PU_DATA[MOTOR_GripperMove])
                  {
                      SMD_PU_DATA[MOTOR_GripperMove] = GripperToOriginPU;
                      SMD_PWM_SetFreqGradient((SMD_Channel)MOTOR_GripperMove,
                                              SMD_PU_DATA[MOTOR_GripperMove],
                                              SMD_ACC_MAX_DEFAULT);
                  }
                  g_sysToOrigin++;
              }
              return;
          case waitToOrigin:
              if (!IN_READ(5))
              {
                  MotorCurStepsU[MOTOR_GripperMove] = 0;
                  MotorCurStepsSub[MOTOR_GripperMove] = 0;
                  OUT(RELAY_WarnYELLOW, WarnLED_off);
                  mbsUSB.regHoldingBuf[OUT_8_ADDR] = WarnLED_off;
                  mbsESP.regHoldingBuf[OUT_8_ADDR] = WarnLED_off;
                  g_sysToOrigin++;
              }
              return;
          case toOriginSuccess:
              break;
          default:
              break;
      }
}
/* ========================= 内部：S曲线正常加减速一步（每通道每1ms） ========================= */
/**
 * @brief  单个通道的"常规"S曲线迭代：把 v_c 向 v_n 推进一步并写入硬件
 * @note   从原 TIM10 ISR 的循环体尾部搬出，逻辑与原来完全一致，只是变成了
 *         一个独立函数，被 SMD_ProcessChannel() 在"不需要换向/不在限位"时调用。
 */
static void SMD_RunSCurve(SMD_Channel ch, SMD_Freq_Gradient *m)
{
    float v_target  = m->v_n;
    float delta_v   = v_target - m->v_c;
    float abs_dv    = (delta_v >= 0.0f) ? delta_v : -delta_v;
    float accel_max = m->acc_max;
    float jerk_val  = g_motorStepsCtl[ch].is_running
                        ? (float)SMD_JERK_DEFAULT
                        : (float)SMD_JERK_DATA[ch];

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
            SMD_PWM_Stop(ch);
            m->v_c        = 0.0f;
            m->a_c        = 0.0f;
            m->is_running = 0;
            return;
        }

        uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
        if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
        if (freq_int > SMD_PWM_FREQ_MAX) freq_int = SMD_PWM_FREQ_MAX;
        SMD_ApplyFreqToHW(ch, freq_int);
        return;
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
    SMD_ApplyFreqToHW(ch, freq_int);
}

/* ========================= 内部：单通道每1ms调度（换向 + 限位 + S曲线） ========================= */
/**
 * @brief  对一个通道执行一次1ms调度，从原 TIM10 ISR 的 for 循环体搬出。
 *         原代码里每个分支的 `continue` 直接对应这里的 `return`（含义完全一致，
 *         因为各通道之间互不依赖，提前结束这个函数等价于原来跳到下一个 ch）。
 *
 *  ┌─────────────────────────────────────────────────────────┐
 *  │ 1. 检查换向标志 dir_change                               │
 *  │    → 若需换向且 v_c != 0：先减速到0                      │
 *  │    → 若需换向且 v_c ≈ 0 ：切换 DR 引脚，清除换向标志      │
 *  │ 2. 检测限位（触发则急停并直接返回）                       │
 *  │ 3. 未换向、未限位 → 调用 SMD_RunSCurve() 正常加减速       │
 *  └─────────────────────────────────────────────────────────┘
 */
static void SMD_ProcessChannel(SMD_Channel ch)
{
    SMD_Freq_Gradient *m = &smd_freq_gradient[ch];
    if (!m->is_running) return;

    /* ================================================================
     * 分支A：v_c == 0，先换向再检测限位
     * ================================================================ */
    if (m->v_c <= 1.0f)
    {
        uint8_t cur_dir = (uint8_t)SMD_DR_READ(ch);

        /* A1. 直接切换方向引脚（无需减速） */
        if (m->dir_change)
        {
            m->v_c        = 0.0f;
            m->a_c        = 0.0f;
            cur_dir       = !cur_dir;
            SMD_DR(ch, cur_dir);
            m->dir_change = 0;
            m->dir_state  = SMD_DIR_NORMAL;
            /* 不 return，继续往下做限位检测再进入S曲线 */
        }

        /* A2. 换向后（或本来就无换向）检测限位 */
        if (SMD_IsLimited((int)ch, cur_dir, m))
        {
            m->dir_change = 0;
            return;
        }

        /* A3. 无限位触发，进入S曲线加速（直接落入下方正常运动逻辑） */
    }
    /* ================================================================
     * 分支B：v_c != 0，先检测限位再决定换向/S曲线
     * ================================================================ */
    else
    {
        uint8_t cur_dir = (uint8_t)SMD_DR_READ(ch);
        if (SMD_IsLimited((int)ch, cur_dir, m))
        {
            return;
        }

        /* 未触发限位，走换向减速或S曲线 */
        if (m->dir_change)
        {
            if (m->dir_state == SMD_DIR_NORMAL)
                m->dir_state = SMD_DIR_DECEL;

            if (m->dir_state == SMD_DIR_DECEL)
            {
                float delta_v = m->v_c;
                if (delta_v <= 1.0f)
                {
                    m->v_c = 0.0f;
                    m->a_c = 0.0f;
                    SMD_PWM_Stop(ch);
                    uint8_t cur_dir2 = (uint8_t)SMD_DR_READ(ch);
                    SMD_DR(ch, !cur_dir2);
                    m->dir_state  = SMD_DIR_WAIT;
                    m->dir_change = 0;
                    SMD_PWM_Start(ch);
                    return;
                }
                SMD_UpdateVelocity(m, delta_v, m->acc_max,
                                   g_motorStepsCtl[ch].is_running
                                     ? (float)SMD_JERK_DEFAULT
                                     : (float)SMD_JERK_DATA[ch]);
                m->v_c -= m->a_c * SMD_UPDATE_DT_s;
                if (m->v_c < 0.0f) m->v_c = 0.0f;
                uint32_t freq_int = (uint32_t)(m->v_c + 0.5f);
                if (freq_int < SMD_PWM_FREQ_MIN) freq_int = SMD_PWM_FREQ_MIN;
                SMD_ApplyFreqToHW(ch, freq_int);
                return;
            }

            if (m->dir_state == SMD_DIR_WAIT)
            {
                m->dir_state = SMD_DIR_NORMAL;
                return;
            }
        }
    }

    /* ---- 未换向、未限位：走正常S曲线运动 ---- */
    SMD_RunSCurve(ch, m);
}

/* ========================= 内部：继电器电机限位安全检测 ========================= */
/**
 * @brief  独立于8路步进电机之外的继电器电机（OUT12/13）限位保护
 *         仅在电机运行时检测，停止状态不触发。逻辑原样从 ISR 尾部搬出。
 */
static void SMD_CheckRelayMotorLimit(void)
{
    if (!OUT_READ(RELAY_MOTOR_1) && !OUT_READ(RELAY_MOTOR_2)) return;

    uint8_t cur_dir = OUT_READ(RELAY_MOTOR_1);  // OUT12=1→正转, OUT12=0→反转
    uint8_t hit = (((!IN_READ(7) || !IN_READ(9)) && cur_dir) || (!IN_READ(8) && !cur_dir));
    if (!IN_READ(19)) hit = 1;

    if (hit)
    {
        OUT(RELAY_MOTOR_1, 0);
        OUT(RELAY_MOTOR_2, 0);
        mbsUSB.regHoldingBuf[OUT_BASE_ADDR + RELAY_MOTOR_1] = 0;
        mbsUSB.regHoldingBuf[OUT_BASE_ADDR + RELAY_MOTOR_2] = 0;
        mbsESP.regHoldingBuf[OUT_BASE_ADDR + RELAY_MOTOR_1] = 0;
        mbsESP.regHoldingBuf[OUT_BASE_ADDR + RELAY_MOTOR_2] = 0;
    }
}

/* ========================= TIM10 1ms中断服务函数（S曲线调度） ========================= */
/**
 * @brief  TIM1更新/TIM10全局中断，1ms周期
 *
 * 移植自ESP32 ideal_motor_smctl_task()，去掉FreeRTOS，改为中断驱动。
 * 每1ms依次：
 *   1. 对每个通道调用 SMD_ProcessChannel()（换向/限位检测 + S曲线一步）
 *   2. 对夹爪/升降/进退三个通道做步数控制决策 SMD_MotorStepsCtl()
 *   3. 推进"回原点"状态机 SMD_SysToOrigin()
 *   4. 继电器电机的独立限位保护 SMD_CheckRelayMotorLimit()
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) == RESET) return;
    __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);

    num++;

    for (int ch = 0; ch < SMD_CH_MAX; ch++)
        SMD_ProcessChannel((SMD_Channel)ch);

    SMD_MotorStepsCtl(MOTOR_GripperMove);
    SMD_MotorStepsCtl(MOTOR_UpDown);
    SMD_MotorStepsCtl(MOTOR_FBack);
    SMD_SysToOrigin();

    SMD_CheckRelayMotorLimit();
}
