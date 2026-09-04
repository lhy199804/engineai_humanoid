# 板载网络控制设计（LCM 指令等效手柄按键）

- 日期：2026-09-03
- 状态：已评审通过（待用户最终确认）
- 硬件：哪吒（NaZha）主控板，星刃人形机器人（ZQ_Biped_SA01）
- 版本基线：V-Hw01.00.00-Fw01.01.00（commit 59e1d04）

## 1. 背景与目标

当前机器人腿部运动与步态切换只能通过手柄按键输入控制。目标：**新增一种控制方式**——外部程序通过以太网 LCM 发送指令到哪吒板，板上控制程序接收后产生与手柄按键输入**完全等效**的效果。手柄原有控制逻辑保持不变。

已确认的需求决策：

| 决策点 | 结论 |
|---|---|
| 触发源 | 外部网络指令驱动（板子程序接收转发，不自主决策） |
| 通信协议 | LCM（复用板上已有 LCM 基础设施） |
| 指令粒度 | 直接写控制字段（消息携带 `rc_control_settings` 的主要字段） |
| 控制权仲裁 | 手柄夺回式：收到网络指令进入网络控制；手柄任意按键按下即夺回 |
| 文档 | README 中 RB+X 与代码不一致的问题本次**不修改** |

## 2. 现有手柄控制链路（背景知识）

手柄 → 哪吒板 → 控制算法共 4 层，本次改动只涉及第 2 层：

1. **手柄读取**：`ZqSA01HardwareBridge::initHardware()` 调 `init_sbus(false)` 打开 SBUS 串口；独立线程 `sbusTask` 每 100ms 调 `run_sbus()` → `receive_sbus()` 收帧（`robot/src/HardwareBridge.cpp`）。
2. **按键解码**：`sbus_packet_complete_logitech()` 把按键组合映射为全局结构体 `rc_control`（`robot/src/rt/rt_rc_interface.cpp:337`）：
   - LB+START→PASSIVE；LB+BACK→OFF；LB+RB→LOCK_JOINT；LB+A→STAND_UP；LB+B→BALANCE_STAND；LB+X（且当前 STAND_UP）→LOCOMOTION
   - LOCOMOTION 内：A 键翻转 `gait_type`（0=站立/自训练模型，1=行走/SA01 出厂模型）；摇杆→`v_des`/`omega_des`
   - 每收到一帧 SBUS 数据都会覆盖写入 `rc_control`（含 `mode`），这是仲裁逻辑必须处理的关键点。
3. **模式传递**：`RobotRunner::setupStep()` 每 2ms 调 `get_rc_control_settings()`（互斥拷贝）→ `ControlFSM::runFSM()` 把 `rc_control.mode` 映射为 `control_mode`，驱动 FSM 状态机（ESTOP/PASSIVE/JOINT_PD/BALANCE_STAND/LOCK_JOINT/RL_LOCOMOTION）。
4. **RL 动作**：`DesiredStateCommand::convertToStateCommands()` 把 `v_des` 等转成期望状态；`LearningBasedController::computeObservation()` 根据 `gait_type`/`still_flag` 切换 STAND/WALK 双 ONNX 模型。

关键结论：下游（FSM、RL 控制器）只消费 `rc_control` 这一个全局结构体，因此网络指令的注入点选在 `rc_control` 写入处，即可与手柄完全等效。

## 3. 总体架构与数据流

```
上位机/PC ──以太网 LCM──▶ 哪吒板 LCM 线程 (handleInterfaceLCM)
                              │ 回调 handleRCControlCommandLCM
                              ▼
                  set_rc_control_from_network()
                              │ 加锁(lcm_get_set_mutex)写 rc_control
                              │ + 置 network_control_active = true
                              ▼
                rc_control 全局结构体 ◀── 手柄线程(每100ms，仲裁后写入)
                              │
               既有链路（不改动）：RobotRunner::setupStep(2ms读)
                → ControlFSM(mode→control_mode) → FSM 状态机 → RL 双模型控制器
```

## 4. 详细设计

### 4.1 新增 LCM 类型

新文件 `EngineAI_Controller/lcm-types/rc_control_command_lcmt.lcm`（仿 `gamepad_lcmt.lcm` 风格）：

```lcm
struct rc_control_command_lcmt {
    double mode;              // RC_mode 枚举: 0=OFF 1=PASSIVE 2=STAND_UP 7=BALANCE_STAND 8=LOCK_JOINT 11=LOCOMOTION
    int32_t gait_type;        // 0=站立(自训练模型) 1=行走(SA01 出厂模型)
    double v_des[3];          // 期望速度 [-1,1]，实际生效 v_des[0..1]
    double omega_des[3];      // 期望角速度 [-1,1]，实际生效 omega_des[2]
}
```

- 通道名：`"rc_control_command"`
- 字段只保留罗技手柄路径实际写入的字段；`p_des`/`rpy_des`/`height_variation`/`step_height` 仅被 Taranis/AT9s 遥控路径使用、本机器人下游不消费，已剔除。数组长度保持 [3] 与 `rc_control_settings` 对齐，便于整体拷贝。
- 用 `scripts/make_types.sh` 重新生成 cpp/java/python 绑定（该脚本会全量再生成，属既有流程）

### 4.2 接收与写入（rt_rc_interface + HardwareBridge）

`robot/src/rt/rt_rc_interface.cpp` 新增：

- 全局标志 `bool network_control_active = false;`（与 `rc_control` 同一互斥锁 `lcm_get_set_mutex` 保护）
- `void set_rc_control_from_network(const rc_control_command_lcmt *msg)`：
  1. **合法性检查（与手柄约束一致）**：`mode` 必须在 {OFF, PASSIVE, STAND_UP, BALANCE_STAND, LOCK_JOINT, LOCOMOTION} 内；`mode == LOCOMOTION` 仅当当前 `rc_control.mode == STAND_UP`（等价手柄 LB+X 的前置条件）。不满足 → 打印警告并忽略整条消息，机器人保持原状态。
  2. `v_des`/`omega_des` 裁剪到 [-1,1]。
  3. 加锁写入 `rc_control` 的 `mode`/`gait_type`/`v_des`/`omega_des`；置 `network_control_active = true`；打印模式切换日志（与手柄路径的日志风格一致）。

`robot/src/HardwareBridge.cpp`：

- `initCommon()` 增加一行：`_interfaceLCM.subscribe("rc_control_command", &HardwareBridge::handleRCControlCommandLCM, this);`
- 新增回调 `handleRCControlCommandLCM(...)`（约 5 行）：转发调用 `set_rc_control_from_network(msg)`。

`robot/include/HardwareBridge.h` 增加回调声明。

### 4.3 仲裁逻辑（手柄夺回）

`robot/src/rt/rt_rc_interface.cpp` 的 `sbus_packet_complete_logitech()` 开头（`update_logitech_data(&data)` 之后）新增：

```cpp
if (network_control_active)
{
    // 任意按键按下 → 手柄夺回控制权（摇杆不参与，避免零漂误触发）
    if (data.A > BUTTERN_PRESS_THRESHOULD || data.B > BUTTERN_PRESS_THRESHOULD ||
        data.X > BUTTERN_PRESS_THRESHOULD || data.Y > BUTTERN_PRESS_THRESHOULD ||
        data.LB > BUTTERN_PRESS_THRESHOULD || data.RB > BUTTERN_PRESS_THRESHOULD ||
        data.START > BUTTERN_PRESS_THRESHOULD || data.BACK > BUTTERN_PRESS_THRESHOULD)
    {
        network_control_active = false;
        printf("[RC] Joystick takes back control\n");
        // 继续执行本函数正常逻辑（本帧即恢复手柄写入）
    }
    else
    {
        return; // 网络控制期间本帧不覆盖 rc_control
    }
}
```

- 夺回后行为与现状完全一致（手柄线程恢复正常每帧写入）。
- 网络控制期间手柄线程仍正常收帧、仅跳过写入，无状态丢失。

### 4.4 错误处理与边界

| 场景 | 行为 |
|---|---|
| 网络无消息 | `rc_control` 保持最后一次网络写入值；不做断连自动回退（YAGNI，后续可扩展超时回退） |
| `mode` 非法 / LOCOMOTION 前置条件不满足 | 拒绝整条消息并打印警告，机器人保持原状态（安全方向） |
| LCM 线程与 SBUS 线程并发 | 写路径全部经 `lcm_get_set_mutex`；`network_control_active` 读侧容忍竞态（最坏晚 100ms 夺回，安全方向） |
| ESTOP 安全 | FSM 的 `safetyPreCheck`/`SafetyChecker` 独立于控制源，网络控制同样受安全保护 |

## 5. 测试计划

1. **编译**：本地 `cmake .. && make -j && make install`（含 lcm 类型重新生成）通过。
2. **板载逐步实测**（每个动作后观察机器人状态与 lcm-spy 关节数据）：
   - `PASSIVE`（开电机）→ `STAND_UP`（直腿站）→ `LOCOMOTION + gait_type=0`（RL 站立）→ `gait_type=1`（行走）→ `gait_type=0`（回站立）→ `LOCK_JOINT`
3. **仲裁验证**：网络控制期间按手柄任意键 → 手柄夺回；再发网络指令 → 重新接管。
4. **安全验证**：ESTOP 下直接发 `LOCOMOTION` → 应被拒绝并打印警告。
5. **测试工具**：新增 `scripts/send_rc_command.py`（lcm-python 发送示例，供上位机参考与测试使用）。

## 6. 涉及文件清单

| 文件 | 改动 |
|---|---|
| `lcm-types/rc_control_command_lcmt.lcm` | 新增 LCM 类型定义 |
| `robot/src/rt/rt_rc_interface.cpp` | 新增 `network_control_active`、`set_rc_control_from_network()`；`sbus_packet_complete_logitech()` 开头加仲裁 |
| `robot/include/rt/rt_rc_interface.h` | 新增函数声明 |
| `robot/src/HardwareBridge.cpp` | 新增通道订阅与回调 |
| `robot/include/HardwareBridge.h` | 新增回调声明 |
| `scripts/send_rc_command.py` | 新增测试发送脚本 |
| `README2.md` | 版本改动记录新增条目（新功能说明、消息格式、通道名） |

手柄主逻辑（`sbus_packet_complete_logitech` 其余部分）一行不动。

## 7. 明确不做（范围外）

- 不修改手柄按键映射与任何手柄行为。
- 不修正 README 中 RB+X 与代码不一致的描述（用户明确指示不动文档）。
- 不做网络断连自动回退、消息确认/心跳等高级特性（后续需要可扩展）。
- 不引入偏置校准（bias）字段到网络消息（校准流程仍由手柄管理）。
