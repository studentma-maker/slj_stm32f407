/*************************************** Copyright (c)******************************************************
** File name            :   mb_hook.h
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
#ifndef _MB_HOOK_H
#define _MB_HOOK_H

/** Includes --------------------------------------------------------------------------------------------*/
#include "main.h"
#include "mb_slave.h"
/**********************************************************************************************************
** Descriptions :  值<->功能码
**
**  WRITE_HOLDING_V 保持不变 所有寄存器使用(控制)
**********************************************************************************************************/
#define WRITE_HOLDING_V   2
/**********************************************************************************************************
** Descriptions :  modbus寄存器地址定义（v2.0 步数控制重构）
**
**  每路电机占用 10 个连续寄存器（基址 = (电机序号-1)*10）：
**  偏移 0: AM   - 报警信号（只读）
**  偏移 1: DR   - 方向控制（读写，写入触发S曲线换向；步数模式下忽略）
**  偏移 2: ACC  - 最大加速度 Hz/s（读写，所有模式下均可更新）
**  偏移 3: JRK  - Jerk Hz/s²（读写，S曲线加加速度）
**  偏移 4: STEP - 步数控制（写入目标步数即进入步数模式）
**                 STEP 在 PU 之前：多寄存器连续写入时先触发步数模式，
**                 后续 PU 写入自动识别为「步数模式→仅更新最大频率」
**  偏移 5: PU   - 目标频率 / 步数模式最大脉冲频率 Hz
**                 非步数模式：写入触发S曲线渐变
**                 步数模式：写入更新最大脉冲频率
**  偏移 6: SP   - 实时速度（读=当前Hz；写0=急停，写N>0=直接跳变）
**  偏移 7~9: 预留
**
**  【步数控制】连续写入 DR→ACC→JRK→STEP→PU→SP
**              STEP 先触发步数模式，PU 感知到 g_motorStepsCtl.is_running=1，
**              走「仅更新最大频率」分支，不再触发 S 曲线
**********************************************************************************************************/
#define SMD_1_AM_ADDR              0
#define SMD_1_DR_ADDR              1
#define SMD_1_ACC_ADDR             2
#define SMD_1_JRK_ADDR             3
#define SMD_1_STEP_ADDR            4
#define SMD_1_PU_ADDR              5
#define SMD_1_SP_ADDR              6

#define SMD_2_AM_ADDR              10
#define SMD_2_DR_ADDR              11
#define SMD_2_ACC_ADDR             12
#define SMD_2_JRK_ADDR             13
#define SMD_2_STEP_ADDR            14
#define SMD_2_PU_ADDR              15
#define SMD_2_SP_ADDR              16

#define SMD_3_AM_ADDR              20
#define SMD_3_DR_ADDR              21
#define SMD_3_ACC_ADDR             22
#define SMD_3_JRK_ADDR             23
#define SMD_3_STEP_ADDR            24
#define SMD_3_PU_ADDR              25
#define SMD_3_SP_ADDR              26

#define SMD_4_AM_ADDR              30
#define SMD_4_DR_ADDR              31
#define SMD_4_ACC_ADDR             32
#define SMD_4_JRK_ADDR             33
#define SMD_4_STEP_ADDR            34
#define SMD_4_PU_ADDR              35
#define SMD_4_SP_ADDR              36

#define SMD_5_AM_ADDR              40
#define SMD_5_DR_ADDR              41
#define SMD_5_ACC_ADDR             42
#define SMD_5_JRK_ADDR             43
#define SMD_5_STEP_ADDR            44
#define SMD_5_PU_ADDR              45
#define SMD_5_SP_ADDR              46

#define SMD_6_AM_ADDR              50
#define SMD_6_DR_ADDR              51
#define SMD_6_ACC_ADDR             52
#define SMD_6_JRK_ADDR             53
#define SMD_6_STEP_ADDR            54
#define SMD_6_PU_ADDR              55
#define SMD_6_SP_ADDR              56

#define SMD_7_AM_ADDR              60
#define SMD_7_DR_ADDR              61
#define SMD_7_ACC_ADDR             62
#define SMD_7_JRK_ADDR             63
#define SMD_7_STEP_ADDR            64
#define SMD_7_PU_ADDR              65
#define SMD_7_SP_ADDR              66

#define SMD_8_AM_ADDR              70
#define SMD_8_DR_ADDR              71
#define SMD_8_ACC_ADDR             72
#define SMD_8_JRK_ADDR             73
#define SMD_8_STEP_ADDR            74
#define SMD_8_PU_ADDR              75
#define SMD_8_SP_ADDR              76

#define OUT_BASE_ADDR			   80
#define OUT_1_ADDR                 80       // 输出信号
#define OUT_2_ADDR                 81       // 输出信号
#define OUT_3_ADDR                 82       // 输出信号
#define OUT_4_ADDR                 83       // 输出信号
#define OUT_5_ADDR                 84       // 输出信号
#define OUT_6_ADDR                 85       // 输出信号
#define OUT_7_ADDR                 86       // 输出信号
#define OUT_8_ADDR                 87       // 输出信号
#define OUT_9_ADDR                 88       // 输出信号
#define OUT_10_ADDR                89       // 输出信号
#define OUT_11_ADDR                90       // 输出信号
#define OUT_12_ADDR                91       // 输出信号
#define OUT_13_ADDR                92       // 输出信号
#define OUT_14_ADDR                93       // 输出信号
#define OUT_15_ADDR                94       // 输出信号
#define OUT_16_ADDR                95       // 输出信号

#define RELAY_MOTOR_1 14
#define RELAY_MOTOR_2 15

#define EC_CLEAR_1_ADDR            96       // 编码器清零
#define EC_CLEAR_2_ADDR            97       // 编码器清零
#define EC_CLEAR_3_ADDR            98       // 编码器清零
#define EC_CLEAR_4_ADDR            99       // 编码器清零
#define EC_CLEAR_5_ADDR            100       // 编码器清零

#define EC_1_ADDR                  101       // 编码器值
#define EC_2_ADDR                  102       // 编码器值
#define EC_3_ADDR                  103       // 编码器值
#define EC_4_ADDR                  104       // 编码器值
#define EC_5_ADDR                  105       // 编码器值

#define ADC_1_ADDR                 106       // ADC值
#define ADC_2_ADDR                 107       // ADC值
#define ADC_3_ADDR                 108       // ADC值
#define ADC_4_ADDR                 109       // ADC值
#define ADC_5_ADDR                 110       // ADC值
#define ADC_6_ADDR                 111       // ADC值

#define IN_1_ADDR                  112        // 输入信号
#define IN_2_ADDR                  113        // 输入信号
#define IN_3_ADDR                  114        // 输入信号
#define IN_4_ADDR                  115        // 输入信号
#define IN_5_ADDR                  116        // 输入信号
#define IN_6_ADDR                  117        // 输入信号
#define IN_7_ADDR                  118        // 输入信号
#define IN_8_ADDR                  119        // 输入信号
#define IN_9_ADDR                  120        // 输入信号
#define IN_10_ADDR                 121        // 输入信号
#define IN_11_ADDR                 122        // 输入信号
#define IN_12_ADDR                 123        // 输入信号
#define IN_13_ADDR                 124        // 输入信号
#define IN_14_ADDR                 125        // 输入信号
#define IN_15_ADDR                 126        // 输入信号
#define IN_16_ADDR                 127        // 输入信号
#define IN_17_ADDR                 128        // 输入信号
#define IN_18_ADDR                 129        // 输入信号
#define IN_19_ADDR                 130        // 输入信号
#define IN_20_ADDR                 131        // 输入信号

#define GRIPPER_CUR_STEPS          132       // 夹爪当前步数
#define SYS_TO_ORIGIN              133       // 系统初始化状态
#define FBACK_CUR_STEPS            134       // 进退电机当前步数


/* 步数控制已统一通过各通道 SMD_x_STEP_ADDR 触发，不再使用独立目标步数寄存器 */
#define STOP_ALL_MOTOR_ADDR        (REG_HOLDING_NREGS - 1) // stop all motor

/**********************************************************************************************************
** Function name        :   mbs_hook_updata_holding
** Descriptions         :   MODBUS更新保存寄存器值
** parameters           :   _mbs: 从机结构体
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_updata_holding(mbs *_mbs);

/**********************************************************************************************************
** Function name        :   mbs_hook_extract_holding
** Descriptions         :   MODBUS提取保存寄存器值
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_extract_holding(mbs *_mbs, uint16_t _reg, uint16_t _val);

/**********************************************************************************************************
** Function name        :   mbs_hook_updata_coils
** Descriptions         :   MODBUS更新线圈状态
** parameters           :   _mbs: 从机结构体
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_updata_coils(mbs *_mbs);

/**********************************************************************************************************
** Function name        :   mbs_hook_extract_coils
** Descriptions         :   MODBUS提取线圈状态
** parameters           :   无
** Returned value       :   无
***********************************************************************************************************/
void mbs_hook_extract_coils(mbs *_mbs, uint16_t _reg, uint8_t _val);

#endif

/********************************************** END OF FILE ***********************************************/
