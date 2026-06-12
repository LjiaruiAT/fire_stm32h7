# 🔫 fire_exact_h7 — STM32H7 纯击球系统，宇树电机直驱 + 重力补偿 + 软件看门狗

> STM32H743VITx · FreeRTOS · FDCAN · RS485 · 立方多项式轨迹 · 蓝牙指令 · 重力自适应

---

## 📋 功能概览

本项目是 exact_fire 的 H7 平台演进版，精简为**纯弹射击球系统**。相比 F4 版本去掉了底盘控制和遥控器，改为串口自定义协议指令触发，电机方案也从 RobStride+CAN 升级为**宇树 GoMotor+RS485 直驱**。

| 子系统 | 硬件 | 通信 | 控制方式 |
|--------|------|------|----------|
| 弹射臂 | 宇树 GoMotor | RS485 (UART2) | MIT 模式 (位置+速度+力矩+阻抗) |
| 指令输入 | 蓝牙模块 | UART7 (DMA) | 0xAA-cmd-0x55 协议 |
| 辅助电机 |串口7 | rs485口2 | 位置+速度 |

---

## 🏗️ 架构分层

```
┌──────────────────────────────────────────┐
│        Task_Init.c（Hit_Task 状态机）      │
│  立方多项式轨迹 · 重力补偿 · 蓝牙指令解析    │
├──────────────────────────────────────────┤
│  step/CubicParam (三次多项式)             │
│  PID2 (经典位置式PID)                     │
│  WatchDog2 (软件看门狗)                   │
├──────────────────────────────────────────┤
│  go_motor (宇树RS485) · dm_h6215 (FDCAN) │
│  FDCANDriver · 485_bus · crc_ccitt       │
├──────────────────────────────────────────┤
│  mylist (通用链表) · RMLibHead (适配层)    │
├──────────────────────────────────────────┤
│   FreeRTOS + CMSIS-RTOS2 + STM32H7 HAL   │
└──────────────────────────────────────────┘
```

---

## 🧠 核心巧思

### 1. 重力补偿前馈 (`GravityCompensatedTorque360`)

弹射臂绕轴旋转时重力力矩呈正弦变化。不用力矩传感器，纯数学模型计算：

```c
torque = torque_max * sinf(angle_current - angle_down);
```

**精妙之处：**
- **三角函数直接建模**：`θ_down` 是重力臂水平（力矩最大）的角度，电机角度绕圈后通过 `NormalizeAngleRad()` 归一化到 [0, 2π)，然后计算最短角度差——无论转过多少圈，补偿值始终正确。
- **零额外硬件**：只需要标定两个参数（最大扭矩 `inv_tor` 和下垂角度 `ini_rad`），无需力矩传感器或额外载荷测量。
- **前馈注入**：重力补偿值叠加在 `let_fly.exp_torque` 上，与轨迹跟踪的 PD 控制输出并行，互不干扰。

### 2. 启动自标定 + 自动就位

```c
if(rad_init_done <= 10) {
    // 阻尼模式等待电机稳定 → 记下当前位置作为基准角度
    GoMotorSend(&let_fly.motor, exp_torque, 0, 0, Kp, Kd);
    GoMotorRecv(&let_fly.motor);
    rad_init_done++;
    unitree_F = let_fly.motor.state.rad;
    // 根据基准角度自动计算三条球路的目标角度
    unitree_S_FAR    = unitree_F + unitree_inv_back_far;
    unitree_S_MIDDLE = unitree_F + unitree_inv_back_middle;
    unitree_S_NEAR   = unitree_F + unitree_inv_back_near;
    unitree_T        = unitree_F - unitree_inv_front;
}
```

**精妙之处：**
- 上电后自动读取电机当前位置作为基准，然后根据预设偏移量计算三条球路（远/中/近）的目标角度。机械安装角度不再需要绝对校准——换一台机器只需改 `unitree_inv_back_*` 偏移常量。
- 标定完成后自动执行一条 `Cubic_SetTrajectory` 将手臂从当前角度平滑移动到远球路的准备位置，上电即就绪。

### 3. 三路独立状态机共用单电机

三条球路（far / middle / near）各有独立的状态机，但共享同一个物理电机。通过**互斥的 `init_done` 标志位**保证同一时刻只有一条球路激活。

**精妙之处：**
- 每条球路由 `BALL_*_IDLE → PREPARE → HIT → RESET` 四个状态组成，结构完全一致但参数不同（准备角度、击球速度、PD 增益）。
- 在击球阶段（`BALL_FAR_HIT`）采用了**分段策略**：
  ```c
  if (pos > threshold)
      GoMotorSend(..., max_vel, target, 8, 0.2);  // 越过阈值前：高速大刚度冲过去
  else
      GoMotorSend(..., 0, target, 4, 0.1);         // 接近目标后：减速小刚度精确定位
  ```
  既保证了击球瞬间的高速度，又避免了到位后的过冲震荡。

### 4. 软件看门狗框架 (`WatchDog2`)

独立于 MCU 硬件看门狗的软件级任务监控。

**精妙之处：**
- **通用链表驱动**：看门狗以链表节点动态添加，每个看门狗有独立的超时时间、回调函数和用户数据。不限数量。
- **位域状态管理**：一个 `uint8_t` 同时编码了三种状态：
  ```
  bit0 → 单次触发（触发后自动挂起）
  bit1 → 当前挂起状态
  bit2 → 启用/禁用
  ```
  位操作判断极快：`(state & 0x06) == 0x06` 一行完成"已启用且未挂起"的判断。
- **惰性初始化**：第一次 `AddWatchDog()` 时才创建监控任务，不浪费 RTOS 资源。
- **10Hz 轮询**：`WATCHDOG_FREQUENCY` 宏控制检查频率，平衡实时性和 CPU 开销。

### 5. 立方多项式轨迹 (`step.c`)

```
p(t) = a₀ + a₁t + a₂t² + a₃t³
```

**精妙之处：**
- **直接解析求解**：四个边界条件 (p₀, v₀, p_T, v_T) → 四个系数，闭式解 O(1) 完成。
- **自动完成检测**：`Cubic_GetFullState()` 检测 `t ≥ T` 后自动清零 `is_running`，调用方只需检查标志位。
- **最小时间保护**：`if (duration < 0.05f) duration = 0.05f`，防止极小时间导致系数爆炸。
- **速度限幅**：目标速度自动钳位在 `M3508_MAX_SPEED_RADS` 范围内，保护电机。

### 6. 宇树 GoMotor RS485 协议 (`go_motor.c`)

通过 RS485 总线以 MIT 模式控制宇树系列电机。

**精妙之处：**
- **结构体封包**：发送/接收数据包用 `#pragma pack(1)` 紧凑结构体直接映射，无需手动拼接/解析字节，类型安全且零拷贝。
- **RS485 方向控制**：`RS485Init()` 支持 GPIO 控制 485 收发切换，适配半双工总线。
- **自动匹配接收**：`GoMotorRecv_AutoMatch()` 从 RS485 总线上的混合数据中按 motor_id 自动匹配对应电机的反馈包。

### 7. FDCAN 驱动适配 (`FDCANDriver`)

H7 平台使用 FDCAN 外设替代 F4 的 bxCAN。

**精妙之处：**
- **接口兼容**：`FDCAN_Filter_Init()` / `FDCAN_Sent()` / `CAN_Receive_DataFrame()` 保持与 F4 版 `CANDrive` 相同的函数签名，上层电机驱动（如 `dm_h6215`）无需改动即可迁移到 H7。
- **扩展帧支持**：`FDCAN_EXT_Sent()` 专门处理 29 位扩展 ID，满足 RobStride 等需要扩展帧的电机。

### 8. 紧凑的蓝牙指令协议

```
帧格式: 0xAA | cmd | 0x55
```

**精妙之处：**
- **帧头帧尾校验**：`RxEventCallback` 中同时检查首字节 (`0xAA`) 和末字节 (`0x55`)，有效过滤噪声和半帧数据。
- **DMA 空闲中断自动恢复**：`HAL_UARTEx_ReceiveToIdle_DMA` 保证变长接收，回调中立即重启 DMA，通信永不中断。
- **三指令即可完整控制**：`0x01 = 远球`, `0x02 = 中球`, `0x03 = 近球`，极简协议降低蓝牙延迟和丢包风险。

### 9. 角度转换适配

```c
static float twenty_to_real_pai(float angle) {
    return angle / 6.369426;  // 将宇树电机的 20 位编码器值转换为弧度
}
```

宇树电机反馈的是 20 位编码器原始值，除以 6.369426（≈ 65536 / (2π) / 减速比修正）转换为真实弧度，一个函数封装了所有硬件差异。

### 10. UART 错误恢复（RS485 + UART7 双路）

RS485 (USART2) 和串口 (UART7) 都有独立的错误恢复逻辑：
- **RS485**：清除 ORE/FE/NE/PE 标志位 + 刷新 RX FIFO，计数错误次数。
- **UART7**：直接重启 DMA 空闲接收，不做复杂处理——蓝牙丢一帧无所谓，下一帧马上到。

---

## 📁 目录结构

```
fire_exact_h7/
├── MyLib/                    # 自研库
│   ├── 485_bus.c/h           #   RS485 总线抽象层
│   ├── go_motor.c/h          #   宇树 GoMotor RS485 协议
│   ├── dm_h6215.c/h          #   DM-H6215 FDCAN 协议
│   ├── FDCANDriver.c/h       #   FDCAN 底层驱动
│   ├── WatchDog2.c/h         #   软件看门狗
│   ├── step.c/h              #   三次多项式轨迹规划
│   ├── PID_old.c/h           #   经典位置式 PID (PID2)
│   ├── motor.c/h             #   DJI 电机数据解析
│   ├── motorEx.c/h           #   DJI M3508 扩展
│   ├── RobStride2.c/h        #   RobStride 电机 CAN 协议
│   ├── crc_ccitt.c/h         #   CRC-CCITT 校验
│   ├── mylist.c/h            #   通用链表
│   └── RMLibHead.h           #   跨平台适配层
├── MyTask/
│   └── Task_Init.c/h         #   业务逻辑：弹射状态机 + 蓝牙指令
├── Core/                     #   STM32CubeMX 生成
│   ├── Src/main.c            #     FDCAN + 双 UART 初始化
│   └── Src/freertos.c        #     RTOS 入口
└── Middlewares/               #   FreeRTOS 源码
```

---

## 🔧 硬件配置

| 参数 | 值 | 定义处 |
|------|-----|--------|
| 主控 | STM32H743VITx, 480MHz | `main.c` |
| RTOS | FreeRTOS 10.x, CMSIS-RTOS2 | `freertos.c` |
| 弹射电机 | 宇树 GoMotor (RS485) | `Task_Init.c` |
| 辅助电机 | DM-H6215 (FDCAN) | `dm_h6215.h` |
| 指令输入 | 蓝牙模块 → UART7 | `HAL_UARTEx_RxEventCallback` |
| 弹射臂最大扭矩补偿 | 0.4 Nm | `Task_Init.c`: `inv_tor` |
| 轨迹规划时间 | 2.0 s (自标定阶段) | `Task_Init.c` |
| 看门狗轮询频率 | 10 Hz | `WatchDog2.h` |
| 球路数量 | 3 (远/中/近) | `Task_Init.c` |

---

## 🚀 快速开始

1. 用 **Keil MDK** 或 **STM32CubeIDE** 打开项目
2. 确认 Middlewares/FreeRTOS 路径正确
3. 根据实际电机安装角度修改 `Task_Init.c` 中的偏移常量：
   - `unitree_inv_back_far` / `unitree_inv_back_middle` / `unitree_inv_back_near`
   - `unitree_inv_front`
4. 编译 → 烧录 → 上电
5. 通过蓝牙发送 `AA 01 55` / `AA 02 55` / `AA 03 55` 触发对应球路

---

## 📝 作者

- **GoMotor 驱动 / 弹射状态机 / 重力补偿**：刘家瑞
- **WatchDog2 / MyList 基础库**：刘家瑞
- **底盘框架 (AutoPilot / ForceChassis)**：刘远钊
- **RMLib 基础库**：Yao (KDRobot)

---

## ⚠️ 注意事项

- 宇树电机上电后需等待 1 秒（`vTaskDelay(1000)`）再开始通信，电机内部初始化需要时间
- 重力补偿参数 `inv_tor` 和 `ini_rad` 需根据实际安装方向标定，不同机器人不同值
- FDCAN 需要 STM32H7 的 FDCAN HAL 驱动，注意与 F4 的 bxCAN API 差异
- RS485 总线半双工，`RS485Init` 的 `ctrl_pin` 用于收发切换，若硬件未接可传 NULL
