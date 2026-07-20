# slj_stm32f407 — 三连机 STM32F407 下位机

> Modbus RTU 从机，负责 8 路步进电机 S 曲线 PWM 控制、16 路继电器输出、20 路数字输入采集、6 路 ADC 采集。

## 项目概览

本项目是**三连机**（上料-折边-缝纫一体化工业机器）的**下位机**固件，运行在 STM32F407（ARM Cortex-M4 @ 168MHz）上。通过 **Modbus RTU（RS-485）** 协议与上位机（Kickpi K7 Qt 应用，见 `slj_kickpi_qt_pro`）通信，接收控制指令并实时上报传感器/电机状态。

- **IDE/工具链**：Keil MDK-ARM（`MDK-ARM/Project.uvprojx`）
- **HAL 库**：STM32F4xx HAL Driver
- **Modbus 协议栈**：自研轻量 Modbus RTU 从机栈（`Modbus_RTU/`）

## 硬件 IO 总览

| 类型 | 数量 | 说明 |
|------|------|------|
| 步进电机 PWM | 8 路 | TIM1/2/3/5/8/9/13/14，含 2 路高级定时器互补输出 |
| 数字输出 (OUT) | 16 路 | 继电器控制（含送发气缸、压发气缸、折边、夹爪、料车固定、三色灯+蜂鸣器） |
| 数字输入 (IN) | 20 路 | 限位开关、料车检测、急停等 |
| 步进电机报警 | 8 路 | SMD_AM_1~8，电机驱动器故障信号 |
| ADC | 6 路 | PC0~PC5 模拟输入 |
| Modbus 串口 | 3 路 | USART1 (USB, 115200), USART2 (ESP, 115200), UART5 (STM, 115200) |

## 目录结构

```
slj_stm32f407/
├── Core/
│   ├── Inc/
│   │   ├── main.h          # IO 引脚宏定义 + 继电器枚举 + IN/OUT/SMD 便捷宏
│   │   ├── SMD.h           # 步进电机驱动头文件（S曲线参数、通道枚举、API）
│   │   ├── gpio.h / tim.h / usart.h / stm32f4xx_it.h
│   │   └── stm32f4xx_hal_conf.h
│   ├── Src/
│   │   ├── main.c          # 入口：初始化 Modbus 三路从机 + SMD PWM，主循环 poll
│   │   ├── SMD.c           # ★ 核心：8路步进电机 S 曲线 PWM 驱动（1079 行）
│   │   ├── gpio.c          # GPIO 初始化（CubeMX 生成）
│   │   ├── tim.c / usart.c / stm32f4xx_it.c / system_stm32f4xx.c
│   │   └── stm32f4xx_hal_msp.c
├── Modbus_RTU/              # 自研 Modbus RTU 从机协议栈
│   ├── mb_slave.h/.c        # 从机状态机（IDLE→RX→EXEC→TX）
│   ├── mb_hook.h/.c         # ★ 寄存器读写回调（映射到硬件动作）
│   ├── mb_port.h/.c         # 硬件抽象层（UART + 3.5T 定时器）
│   └── mb_crc.h/.c          # CRC-16 校验
├── Drivers/                 # STM32 HAL 库 + CMSIS
│   ├── STM32F4xx_HAL_Driver/
│   └── CMSIS/
└── MDK-ARM/                 # Keil 工程文件 + 启动文件
    ├── startup_stm32f407xx.s
    └── JLinkSettings.ini
```

## 核心模块详解

### 1. `Core/Src/SMD.c` — 8 路步进电机 S 曲线 PWM 驱动（核心文件）

这是整个下位机最关键的模块，实现 8 路步进电机的独立 S 曲线加减速控制。

**通道分配：**

| 通道 | 别名 | 定时器 | 引脚 | 功能 | 步数控制 |
|------|------|--------|------|------|----------|
| CH0 | MOTOR_PaiFa | TIM2_CH2 | PA1 | 排发电机 | — |
| CH1 | MOTOR_Trans | TIM9_CH1 | PA2 | 送发（输送）电机 | — |
| CH2 | MOTOR_FBack | TIM5_CH4 | PA3 | 进退电机 | ✅ |
| CH3 | MOTOR_UpDown | TIM8_CH1N | PA5 | 升降电机 | ✅ |
| CH4 | MOTOR_GripperMove | TIM13_CH1 | PA6 | 夹爪移动电机 | ✅ |
| CH5 | MOTOR_Feed | TIM14_CH1 | PA7 | 上料电机 | — |
| CH6 | — | TIM1_CH2N | PB0 | 预留 | — |
| CH7 | — | TIM3_CH4 | PB1 | 预留 | — |

**S 曲线算法** （移植自 ESP32 `motor_smctl_t`）：
- 每通道维护独立运动状态 `SMD_Freq_Gradient`：当前速度 `v_c`、加速度 `a_c`、目标速度 `v_n`、jerk
- TIM10 每 1ms 中断一次，遍历 8 通道执行 S 曲线迭代
- 加速段：以 jerk (Hz/s²) 速率持续增大加速度，上限为 ACC_MAX (Hz/s)
- 减速段：当剩余速差不足以平滑制动时，以计算出的 jerk 降低加速度
- 目标为 1Hz 时自动停止 PWM（"S 曲线减速到停"语义），避免 1Hz 蠕动

**关键参数：**
- 频率范围：1 ~ 8000 Hz
- 计数器频率：1 MHz
- ACC（最大加速度）范围：1 ~ 8000 Hz/s（最近提交提高了上限）
- Jerk 范围：1 ~ 15000 Hz/s²

**步数控制** （CH2/CH3/CH4）：
- `GripperStepsCtl_t` 结构体管理：`is_running`／`targetSteps`／`braking`
- 支持两种模式：
  - **正常模式**：给定目标步数，电机自动判断方向、加速→巡航→预判刹车距离→减速→精确停止
  - **限位直达模式**：targetSteps=0 向原点限位运行，targetSteps=0xFFFF 向远端限位运行
- 刹车算法：`SMD_CalcAccNeedSteps()` 基于当前速度/加速度/jerk 正向计算 S 曲线减速所需步数，提前刹车到 100Hz 巡航，最后精确停到目标
- **方向反逻辑**：FBack 电机 DR=0 时步数增加（与常规逻辑相反）

**限位保护** (`SMD_IsLimited`)：
- 升降电机：IN1(0) 上升限位
- 进退电机：IN2(1) 和 IN3(2) 双向限位
- 夹爪电机：IN6(5) 和 IN7(6) 双向限位
- 急停：IN20(19)
- 触发限位时立即停止 PWM，清除运动状态；夹爪触发原点限位时步数清零

**换向保护**：
- 写 DR 寄存器不直接切换 GPIO，而是设置 `dir_change` 标志
- 中断先减速到 0 → 切换方向引脚 → 再加速（S 曲线换向，避免失步）

**系统复位** (`SMD_SysToOrigin`)：
- 上电自动执行 5 阶段复位状态机：初始化继电器 → 设置夹爪方向 → 等待气缸升起 → 发夹爪原点脉冲 → 等待到达原点

### 2. `Modbus_RTU/` — Modbus RTU 从机协议栈

**三路独立从机**：

| 从机 | 串口 | 波特率 | 从机地址 | 用途 |
|------|------|--------|----------|------|
| `mbsUSB` | USART1 | 115200 | 0x01 | 与上位机 Kickpi K7 通信（主通道） |
| `mbsESP` | USART2 | 115200 | 0x01 | 预留（ESP 无线模块） |
| `mbsSTM` | UART5 | 115200 | 0x01 | 与另一 STM32 通信（传感器采集） |

**协议栈架构**：
- `mb_slave.c`：状态机驱动，IDLE → RX（接收） → RX_CHECK（CRC 校验） → EXEC（执行回调） → TX（发送响应）
- `mb_hook.c`：寄存器读写回调，将 Modbus 寄存器操作映射到硬件动作
- `mb_port.c`：硬件抽象层（UART 中断收发 + 3.5T 定时器）
- 支持功能码：0x03（读保持寄存器）、0x06（写单个寄存器）、0x10（写多个寄存器）、0x01/0x05（线圈）

### 3. Modbus 寄存器映射（与上位机 `reg_map.h` 一致）

寄存器空间为 200 个 16-bit 保持寄存器（0~199）：

#### 步进电机控制区（0~79）：每路 10 个寄存器

| 偏移 | 名称 | 读写 | 说明 |
|------|------|------|------|
| +0 | AM | R | 报警信号（电机驱动器故障） |
| +1 | EN | R/W | 使能控制（直接写 GPIO） |
| +2 | DR | R/W | 方向控制（写触发 S 曲线换向） |
| +3 | PU | R/W | 目标频率 Hz（写触发 S 曲线规划） |
| +4 | ACC | R/W | 最大加速度 Hz/s |
| +5 | JRK | R/W | Jerk Hz/s²（S 曲线柔和度，写 0 恢复默认） |
| +6 | SP | R/W | 实时速度（读=当前Hz；写 0=急停，写 N>1=跳变） |
| +7~9 | — | — | 预留 |

#### IO 映射区（80~131）

| 地址 | 说明 |
|------|------|
| 80~95 | 16 路继电器输出（OUT1~OUT16） |
| 96~100 | 编码器清零命令（写入触发转发给 STM 从机） |
| 101~105 | 编码器计数值（从 STM 从机读取） |
| 106~125 | 20 路数字输入（IN1~IN20） |
| 126~131 | 6 路 ADC 值（从 STM 从机读取） |

#### 扩展状态区（132~199）

| 地址 | 名称 | 说明 |
|------|------|------|
| 132 | GRIPPER_CUR_STEPS | 夹爪电机当前步数 |
| 133 | SYS_TO_ORIGIN | 系统复位状态（0~5 对应 GrippertoOrigin_P 枚举） |
| 134 | UPDOWN_CUR_STEPS | 升降电机当前步数 |
| 135 | FBACK_CUR_STEPS | 进退电机当前步数 |
| 196 | FBACK_TARGET_STEPS | 进退电机目标步数（写触发步数控制） |
| 197 | UPDOWN_TARGET_STEPS | 升降电机目标步数（写触发步数控制） |
| 198 | GRIPPER_TARGET_STEPS | 夹爪电机目标步数（写触发步数控制） |
| 199 | STOP_ALL_MOTOR | 全部电机急停（写 1 触发） |

### 4. 继电器映射（`main.h` RelayID 枚举）

| 索引 | 别名 | 功能 |
|------|------|------|
| RELAY_1 | OUT1 | 备用 |
| RELAY_2 | FeedHair | 送发气缸（0=下降/1=上升，注意语义可能与物理相反） |
| RELAY_3 | PressHair | 压发气缸 |
| RELAY_4 | EdgeFold | 折边气缸 |
| RELAY_5 | GripClose | 夹爪闭合 |
| RELAY_6 | EDGEFOLD | 折边（第二路） |
| RELAY_7 | WarnRED | 红色警示灯 |
| RELAY_8 | WarnYELLOW | 黄色警示灯 |
| RELAY_9 | WarnGREEN | 绿色警示灯 |
| RELAY_10 | WarnBEEP | 蜂鸣器 |
| RELAY_11 | CartFixing | 料车固定 |
| RELAY_12~16 | OUT12~16 | 备用 / 继电器电机控制 |

其中 OUT12/OUT13 用于继电器控制的普通电机（非步进），TIM10 中断中有独立的限位检测逻辑（IN7/IN8/IN9）。

### 5. 数字输入映射（IN1~IN20）

关键输入信号（由上位机业务逻辑定义）：
- IN1(0)：升降电机上限位
- IN2(1)、IN3(2)：进退电机双向限位
- IN4(3)、IN5(4)：送发/压发气缸到位检测
- IN6(5)、IN7(6)：夹爪电机双向限位
- IN8(7)、IN9(8)、IN10(9)：继电器电机限位
- IN20(19)：**急停按钮**（全局，触发时停止所有电机+继电器电机）

## 数据流

```
┌────────────────────────────────────────────────────────────┐
│  上位机 (Kickpi K7 Qt)                                      │
│  Modbus RTU Master @ 115200 bps                            │
│  通过 CH9344 USB 转 RS-485 → /dev/ch9344_0                 │
└──────────┬─────────────────────────────────────────────────┘
           │ Modbus RTU (RS-485)
           ▼
┌────────────────────────────────────────────────────────────┐
│  STM32F407 (本项目)                                         │
│                                                            │
│  USART1 (mbsUSB) ← 主通信口                                 │
│     │                                                       │
│     ├─ mb_slave 状态机 ─→ mb_hook 回调                       │
│     │                                                       │
│     ├─ 读寄存器：实时采集 IN/OUT/AM/PU/ACC/JRK/SP/步数       │
│     │                                                       │
│     └─ 写寄存器：触发硬件动作                                 │
│          ├─ EN → GPIO 直接控制                               │
│          ├─ DR → S 曲线换向 (dir_change)                     │
│          ├─ PU/ACC → S 曲线重规划                            │
│          ├─ JRK → 更新 jerk 参数                             │
│          ├─ SP → 急停 / 跳变                                 │
│          ├─ TARGET_STEPS → 步数控制                          │
│          └─ OUT → 继电器 GPIO                               │
│                                                            │
│  UART5 (mbsSTM) ← 传感器子 MCU 通信 (115200)                │
│     └─ 转发 ADC + 编码器数据到 USB/ESP 从机                  │
│                                                            │
│  TIM10 1ms 中断                                             │
│     ├─ 8 路 S 曲线速度迭代                                   │
│     ├─ 3 路步数控制 (夹爪/升降/进退)                          │
│     ├─ 系统复位状态机                                        │
│     └─ 继电器电机限位检测                                    │
└────────────────────────────────────────────────────────────┘
```

## 关键设计决策

- **S 曲线替代梯形加减速**：避免步进电机失步，加速度连续变化（jerk 可控），运行更平滑
- **中断驱动而非 RTOS**：所有逻辑在 TIM10 1ms 中断中完成，无 RTOS 依赖，降低复杂度
- **三路 Modbus 从机**：USB 口与上位机通信，ESP 口预留无线，STM 口级联传感器子 MCU；寄存器镜像在 `mbs_hook_updata_holding()` 中实时刷新
- **步数控制预判刹车**：`SMD_CalcAccNeedSteps()` 基于当前运动状态正向推算 S 曲线减速所需步数，避免过冲
- **WRITE_HOLDING_V（=2）保护**：上位机批量写寄存器时用值 2 标记"保持不变"，下位机收到 2 直接跳过，避免误改
- **子步累加消除浮点误差**：步数使用 1000 子步/整步的定点累加，避免浮点积分累积误差

## 编译

在 Keil MDK-ARM 中打开 `MDK-ARM/Project.uvprojx`，编译下载到 STM32F407。

- 编译器：ARM Compiler 5/6
- 依赖：STM32F4xx HAL Driver、CMSIS
- 无需外部库依赖（Modbus 协议栈自包含）
