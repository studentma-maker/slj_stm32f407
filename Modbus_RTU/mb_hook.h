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
** Descriptions :  modbus寄存器地址定义
**
**  每路电机占用 10 个连续寄存器（基址 = (电机序号-1)*10）：
**  偏移 0: AM  - 报警信号（只读）
**  偏移 1: EN  - 使能控制（读写）
**  偏移 2: DR  - 方向控制（读写，写入触发S曲线换向）
**  偏移 3: PU  - 目标频率 Hz（读写，写入触发S曲线重新规划）
**  偏移 4: ACC - 最大加速度 Hz/s（读写，写入触发S曲线重新规划）
**  偏移 5: JRK - Jerk Hz/s²（读写，S曲线加加速度，用户可自定义）
**  偏移 6: SP  - 实时速度（读：当前Hz；写：直接跳变或急停，0=急停）
**  偏移 7~9: 预留
**********************************************************************************************************/
#define SMD_1_AM_ADDR              0        // 报警信号（只读）
#define SMD_1_EN_ADDR              1        // 使能控制
#define SMD_1_DR_ADDR              2        // 方向控制
#define SMD_1_PU_ADDR              3        // 目标频率
#define SMD_1_ACC_ADDR             4        // 加速度最大值
#define SMD_1_JRK_ADDR             5        // Jerk控制（S曲线加加速度）
#define SMD_1_SP_ADDR              6        // 实时速度（写0=急停，写非0无效）

#define SMD_2_AM_ADDR              10       // 报警信号（只读）
#define SMD_2_EN_ADDR              11       // 使能控制
#define SMD_2_DR_ADDR              12       // 方向控制
#define SMD_2_PU_ADDR              13       // 目标频率
#define SMD_2_ACC_ADDR             14       // 加速度最大值
#define SMD_2_JRK_ADDR             15       // Jerk控制
#define SMD_2_SP_ADDR              16       // 实时速度（写0=急停，写非0无效）

#define SMD_3_AM_ADDR              20       // 报警信号（只读）
#define SMD_3_EN_ADDR              21       // 使能控制
#define SMD_3_DR_ADDR              22       // 方向控制
#define SMD_3_PU_ADDR              23       // 目标频率
#define SMD_3_ACC_ADDR             24       // 加速度最大值
#define SMD_3_JRK_ADDR             25       // Jerk控制
#define SMD_3_SP_ADDR              26       // 实时速度（写0=急停，写非0无效）

#define SMD_4_AM_ADDR              30       // 报警信号（只读）
#define SMD_4_EN_ADDR              31       // 使能控制
#define SMD_4_DR_ADDR              32       // 方向控制
#define SMD_4_PU_ADDR              33       // 目标频率
#define SMD_4_ACC_ADDR             34       // 加速度最大值
#define SMD_4_JRK_ADDR             35       // Jerk控制
#define SMD_4_SP_ADDR              36       // 实时速度（写0=急停，写非0无效）

#define SMD_5_AM_ADDR              40       // 报警信号（只读）
#define SMD_5_EN_ADDR              41       // 使能控制
#define SMD_5_DR_ADDR              42       // 方向控制
#define SMD_5_PU_ADDR              43       // 目标频率
#define SMD_5_ACC_ADDR             44       // 加速度最大值
#define SMD_5_JRK_ADDR             45       // Jerk控制
#define SMD_5_SP_ADDR              46       // 实时速度（写0=急停，写非0无效）

#define SMD_6_AM_ADDR              50       // 报警信号（只读）
#define SMD_6_EN_ADDR              51       // 使能控制
#define SMD_6_DR_ADDR              52       // 方向控制
#define SMD_6_PU_ADDR              53       // 目标频率
#define SMD_6_ACC_ADDR             54       // 加速度最大值
#define SMD_6_JRK_ADDR             55       // Jerk控制
#define SMD_6_SP_ADDR              56       // 实时速度（写0=急停，写非0无效）

#define SMD_7_AM_ADDR              60       // 报警信号（只读）
#define SMD_7_EN_ADDR              61       // 使能控制
#define SMD_7_DR_ADDR              62       // 方向控制
#define SMD_7_PU_ADDR              63       // 目标频率
#define SMD_7_ACC_ADDR             64       // 加速度最大值
#define SMD_7_JRK_ADDR             65       // Jerk控制
#define SMD_7_SP_ADDR              66       // 实时速度（写0=急停，写非0无效）

#define SMD_8_AM_ADDR              70       // 报警信号（只读）
#define SMD_8_EN_ADDR              71       // 使能控制
#define SMD_8_DR_ADDR              72       // 方向控制
#define SMD_8_PU_ADDR              73       // 目标频率
#define SMD_8_ACC_ADDR             74       // 加速度最大值
#define SMD_8_JRK_ADDR             75       // Jerk控制
#define SMD_8_SP_ADDR              76       // 实时速度（写0=急停，写非0无效）

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

#define IN_1_ADDR                  106        // 输入信号
#define IN_2_ADDR                  107        // 输入信号
#define IN_3_ADDR                  108        // 输入信号
#define IN_4_ADDR                  109        // 输入信号
#define IN_5_ADDR                  110        // 输入信号
#define IN_6_ADDR                  111        // 输入信号
#define IN_7_ADDR                  112        // 输入信号
#define IN_8_ADDR                  113        // 输入信号
#define IN_9_ADDR                  114        // 输入信号
#define IN_10_ADDR                 115        // 输入信号
#define IN_11_ADDR                 116        // 输入信号
#define IN_12_ADDR                 117        // 输入信号
#define IN_13_ADDR                 118        // 输入信号
#define IN_14_ADDR                 119        // 输入信号
#define IN_15_ADDR                 120        // 输入信号
#define IN_16_ADDR                 121        // 输入信号
#define IN_17_ADDR                 122        // 输入信号
#define IN_18_ADDR                 123        // 输入信号
#define IN_19_ADDR                 124        // 输入信号
#define IN_20_ADDR                 125        // 输入信号

#define ADC_1_ADDR                 126       // ADC值
#define ADC_2_ADDR                 127       // ADC值
#define ADC_3_ADDR                 128       // ADC值
#define ADC_4_ADDR                 129       // ADC值
#define ADC_5_ADDR                 130       // ADC值
#define ADC_6_ADDR                 131       // ADC值


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
