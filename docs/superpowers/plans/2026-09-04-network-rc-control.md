# 板载网络控制（LCM 指令等效手柄按键）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在保留手柄控制逻辑不变的前提下，新增网络控制方式：外部程序经以太网 LCM 发送 `rc_control_command_lcmt` 消息，哪吒板写入全局 `rc_control`，效果与手柄按键完全等效，手柄任意按键按下即夺回控制权。

**Architecture:** 注入点选在 `rc_control` 全局结构体写入处（手柄按键解码的终点）。校验/裁剪逻辑抽成纯函数文件并附离线单元测试；LCM 回调经 `set_rc_control_from_network()` 加锁写入；`sbus_packet_complete_logitech()` 开头加仲裁判断（网络控制期间手柄线程只监控按键、不覆盖）。

**Tech Stack:** C++11、LCM 1.5.0（lcm-gen）、pthread 互斥、CMake、python-lcm（测试脚本）。

**Spec:** `docs/superpowers/specs/2026-09-03-network-rc-control-design.md`

---

### Task 0: 环境准备确认

**Files:** 无改动

- [ ] **Step 1: 确认 lcm-gen 与构建目录可用**

Run: `lcm-gen --version && ls EngineAI_Controller/build`
Expected: lcm-gen 版本输出；build 目录存在（历史构建目录）

- [ ] **Step 2: 确认工作区干净**

Run: `git status`
Expected: 干净（或仅本计划的文档变更）

---

### Task 1: 新增 LCM 类型并重新生成绑定

**Files:**
- Create: `EngineAI_Controller/lcm-types/rc_control_command_lcmt.lcm`

- [ ] **Step 1: 创建 LCM 类型定义文件**

```lcm
struct rc_control_command_lcmt {
    double mode;              // RC_mode 枚举: 0=OFF 1=PASSIVE 2=STAND_UP 7=BALANCE_STAND 8=LOCK_JOINT 11=LOCOMOTION
    int32_t gait_type;        // 0=站立(自训练模型) 1=行走(SA01 出厂模型)，仅 LOCOMOTION 模式生效
    double v_des[3];          // 期望速度 [-1,1]，实际生效 v_des[0..1]
    double omega_des[3];      // 期望角速度 [-1,1]，实际生效 omega_des[2]
}
```

- [ ] **Step 2: 重新生成 cpp/java/python 绑定**

Run: `cd EngineAI_Controller/scripts && bash make_types.sh`
Expected: 输出 Done with LCM type generation；`lcm-types/cpp/rc_control_command_lcmt.hpp` 存在

- [ ] **Step 3: 确认生成的 C++ 类型与字段**

Run: `ls ../lcm-types/cpp/rc_control_command_lcmt.hpp && grep -n "mode\|gait_type\|v_des\|omega_des" ../lcm-types/cpp/rc_control_command_lcmt.hpp | head -10`
Expected: 文件存在，四个字段均出现在结构体定义中

- [ ] **Step 4: 提交**

```bash
git add -A lcm-types/
git commit -s -m "[New]: 新增rc_control_command_lcmt消息类型及生成绑定

1. 网络控制指令消息：mode/gait_type/v_des/omega_des
2. 重新生成cpp/java/python绑定"
```

---

### Task 2: 校验/裁剪纯逻辑（TDD）

**Files:**
- Create: `EngineAI_Controller/robot/include/rt/rt_network_command.h`
- Create: `EngineAI_Controller/robot/src/rt/rt_network_command.cpp`
- Test: `EngineAI_Controller/tests/test_rt_network_command.cpp`

- [ ] **Step 1: 写失败测试**

创建 `EngineAI_Controller/tests/test_rt_network_command.cpp`：

```cpp
#include <cstdio>
#include "rt/rt_network_command.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do                                                                     \
    {                                                                      \
        if (!(cond))                                                       \
        {                                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static rc_control_command_lcmt make_msg(double mode, int gait)
{
    rc_control_command_lcmt msg;
    msg.mode = mode;
    msg.gait_type = gait;
    for (int i = 0; i < 3; i++)
    {
        msg.v_des[i] = 0.0;
        msg.omega_des[i] = 0.0;
    }
    return msg;
}

int main()
{
    // ---- validate: mode 合法性 ----
    CHECK(validate_rc_network_command(make_msg(0.0, 0), 0.0) == 0);  // OFF 总是合法
    CHECK(validate_rc_network_command(make_msg(1.0, 0), 0.0) == 0);  // PASSIVE
    CHECK(validate_rc_network_command(make_msg(2.0, 0), 0.0) == 0);  // STAND_UP
    CHECK(validate_rc_network_command(make_msg(99.0, 0), 0.0) == -1); // 非法 mode

    // ---- validate: LOCOMOTION 前置条件 ----
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 2.0) == 0);  // STAND_UP -> LOCOMOTION 允许
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 0.0) == -2); // OFF -> LOCOMOTION 拒绝
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 11.0) == -2); // LOCOMOTION -> LOCOMOTION 拒绝(与手柄LB+X一致)

    // ---- clamp: 裁剪到 [-1,1] ----
    {
        rc_control_command_lcmt msg = make_msg(11.0, 0);
        msg.v_des[0] = 1.5;
        msg.v_des[1] = 0.3;
        msg.omega_des[1] = -2.5;
        clamp_rc_network_command(msg);
        CHECK(msg.v_des[0] == 1.0);
        CHECK(msg.v_des[1] == 0.3);
        CHECK(msg.omega_des[1] == -1.0);
    }

    // ---- clamp: gait_type 归一化 0/1 ----
    {
        rc_control_command_lcmt msg = make_msg(11.0, 5);
        clamp_rc_network_command(msg);
        CHECK(msg.gait_type == 1);

        msg.gait_type = -3;
        clamp_rc_network_command(msg);
        CHECK(msg.gait_type == 1);

        msg.gait_type = 0;
        clamp_rc_network_command(msg);
        CHECK(msg.gait_type == 0);
    }

    if (g_failures == 0)
    {
        printf("ALL TESTS PASSED\n");
        return 0;
    }
    printf("%d CHECK(S) FAILED\n", g_failures);
    return 1;
}
```

- [ ] **Step 2: 运行测试确认失败（头文件尚不存在）**

Run: `cd EngineAI_Controller && g++ -std=c++11 -I robot/include -I lcm-types/cpp tests/test_rt_network_command.cpp robot/src/rt/rt_network_command.cpp -o build/test_rt_network_command`
Expected: 编译失败 `rt/rt_network_command.h: No such file or directory`

- [ ] **Step 3: 实现纯逻辑**

创建 `EngineAI_Controller/robot/include/rt/rt_network_command.h`：

```cpp
//
// Created for network rc control (2026/09)
//

#ifndef ZQ_HUMANOID_RT_NETWORK_COMMAND_H
#define ZQ_HUMANOID_RT_NETWORK_COMMAND_H

#include "rc_control_command_lcmt.hpp"

/**
 * 校验网络控制指令合法性（纯函数，可离线测试）
 * @return 0 合法; -1 mode 非法; -2 LOCOMOTION 前置条件不满足(当前非 STAND_UP)
 */
int validate_rc_network_command(const rc_control_command_lcmt &msg, double current_mode);

/**
 * v_des/omega_des 各分量裁剪到 [-1,1]；gait_type 归一化为 0/1（0=站立，非0=行走）
 */
void clamp_rc_network_command(rc_control_command_lcmt &msg);

#endif // ZQ_HUMANOID_RT_NETWORK_COMMAND_H
```

创建 `EngineAI_Controller/robot/src/rt/rt_network_command.cpp`：

```cpp
#include "rt/rt_network_command.h"

#include <cstddef>

#include "rt/rt_rc_interface.h"

namespace
{
constexpr double CMD_LIMIT = 1.0;

bool is_legal_mode(double mode)
{
    return mode == RC_mode::OFF || mode == RC_mode::PASSIVE ||
           mode == RC_mode::STAND_UP || mode == RC_mode::BALANCE_STAND ||
           mode == RC_mode::LOCK_JOINT || mode == RC_mode::LOCOMOTION;
}
} // namespace

int validate_rc_network_command(const rc_control_command_lcmt &msg, double current_mode)
{
    if (!is_legal_mode(msg.mode))
    {
        return -1;
    }
    if (msg.mode == RC_mode::LOCOMOTION && current_mode != RC_mode::STAND_UP)
    {
        return -2;
    }
    return 0;
}

void clamp_rc_network_command(rc_control_command_lcmt &msg)
{
    for (int i = 0; i < 3; i++)
    {
        if (msg.v_des[i] > CMD_LIMIT)
            msg.v_des[i] = CMD_LIMIT;
        else if (msg.v_des[i] < -CMD_LIMIT)
            msg.v_des[i] = -CMD_LIMIT;

        if (msg.omega_des[i] > CMD_LIMIT)
            msg.omega_des[i] = CMD_LIMIT;
        else if (msg.omega_des[i] < -CMD_LIMIT)
            msg.omega_des[i] = -CMD_LIMIT;
    }

    if (msg.gait_type != 0)
    {
        msg.gait_type = 1;
    }
}
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd EngineAI_Controller && g++ -std=c++11 -I robot/include -I lcm-types/cpp tests/test_rt_network_command.cpp robot/src/rt/rt_network_command.cpp -o build/test_rt_network_command && ./build/test_rt_network_command`
Expected: 输出 `ALL TESTS PASSED`，退出码 0

- [ ] **Step 5: 提交**

```bash
git add robot/include/rt/rt_network_command.h robot/src/rt/rt_network_command.cpp tests/test_rt_network_command.cpp
git commit -s -m "[New]: 新增网络指令校验与裁剪纯逻辑及单元测试

1. validate_rc_network_command: mode合法性+LOCOMOTION前置条件(与手柄LB+X一致)
2. clamp_rc_network_command: v_des/omega_des裁剪[-1,1]，gait_type归一化0/1"
```

---

### Task 3: rt_rc_interface 新增网络写入函数与标志

**Files:**
- Modify: `EngineAI_Controller/robot/include/rt/rt_rc_interface.h`（第 72 行 `get_rc_control_settings` 声明之后）
- Modify: `EngineAI_Controller/robot/src/rt/rt_rc_interface.cpp`（头部 include 区与 `get_rc_control_settings` 函数之后）

- [ ] **Step 1: 头文件新增声明**

在 `rt_rc_interface.h` 中 `void get_rc_control_settings(void *settings);` 之后追加：

```cpp
void set_rc_control_from_network(const void *msg);
```

- [ ] **Step 2: 源文件新增 include**

在 `rt_rc_interface.cpp` 头部（`#include "Utilities/EdgeTrigger.h"` 之后）追加：

```cpp
#include "rc_control_command_lcmt.hpp"
#include "rt/rt_network_command.h"
```

- [ ] **Step 3: 源文件新增全局标志与写入函数**

在 `rt_rc_interface.cpp` 的 `get_rc_control_settings` 函数定义之后（约第 26 行）追加：

```cpp
// 网络控制权标志: true 表示当前由网络指令控制，手柄线程只监控按键不覆盖 rc_control
bool network_control_active = false;

void set_rc_control_from_network(const void *msg)
{
    const rc_control_command_lcmt *cmd = static_cast<const rc_control_command_lcmt *>(msg);

    rc_control_command_lcmt clamped = *cmd;
    clamp_rc_network_command(clamped);

    pthread_mutex_lock(&lcm_get_set_mutex);

    int ret = validate_rc_network_command(clamped, rc_control.mode);
    if (ret == 0)
    {
        rc_control.mode = clamped.mode;
        // 与手柄一致: 摇杆量/步态仅在 LOCOMOTION 模式下生效
        if (rc_control.mode == RC_mode::LOCOMOTION)
        {
            rc_control.gait_type = clamped.gait_type;
            rc_control.v_des[0] = clamped.v_des[0];
            rc_control.v_des[1] = clamped.v_des[1];
            rc_control.v_des[2] = clamped.v_des[2];
            rc_control.omega_des[0] = clamped.omega_des[0];
            rc_control.omega_des[1] = clamped.omega_des[1];
            rc_control.omega_des[2] = clamped.omega_des[2];
        }
        network_control_active = true;
        printf("[RC] Network command applied: mode=%.0f gait_type=%d\n",
               rc_control.mode, rc_control.gait_type);
    }
    else
    {
        printf("[RC] Network command REJECTED (ret=%d, current mode=%.0f)\n",
               ret, rc_control.mode);
    }

    pthread_mutex_unlock(&lcm_get_set_mutex);
}
```

- [ ] **Step 4: 编译 robot 库目标验证**

Run: `cd EngineAI_Controller/build && cmake .. > /dev/null && make robot -j$(nproc)`
Expected: 编译通过无错误（新文件被 `file(GLOB ... src/rt/*.cpp)` 自动纳入）

- [ ] **Step 5: 提交**

```bash
git add robot/include/rt/rt_rc_interface.h robot/src/rt/rt_rc_interface.cpp
git commit -s -m "[New]: rt_rc_interface新增网络指令写入函数set_rc_control_from_network

1. LCM消息经校验裁剪后加锁写入rc_control，置network_control_active
2. 摇杆量/步态仅LOCOMOTION模式生效，与手柄行为一致"
```

---

### Task 4: 手柄夺回仲裁

**Files:**
- Modify: `EngineAI_Controller/robot/src/rt/rt_rc_interface.cpp`（`sbus_packet_complete_logitech()` 开头，约第 340 行 `update_logitech_data(&data);` 之后）

- [ ] **Step 1: 插入仲裁判断**

在 `sbus_packet_complete_logitech()` 中，`logitech_data data; update_logitech_data(&data);` 之后、原有 `if (data.LB ...)` 模式判断之前插入：

```cpp
    if (network_control_active)
    {
        // 网络控制期间手柄线程只监控按键；任意按键按下 -> 手柄夺回控制权
        if (data.A > BUTTERN_PRESS_THRESHOULD || data.B > BUTTERN_PRESS_THRESHOULD ||
            data.X > BUTTERN_PRESS_THRESHOULD || data.Y > BUTTERN_PRESS_THRESHOULD ||
            data.LB > BUTTERN_PRESS_THRESHOULD || data.RB > BUTTERN_PRESS_THRESHOULD ||
            data.START > BUTTERN_PRESS_THRESHOULD || data.BACK > BUTTERN_PRESS_THRESHOULD)
        {
            network_control_active = false;
            printf("[RC] Joystick takes back control\n");
        }
        else
        {
            return; // 本帧不覆盖 rc_control，保持网络最后写入值
        }
    }
```

- [ ] **Step 2: 编译 robot 库目标验证**

Run: `cd EngineAI_Controller/build && make robot -j$(nproc)`
Expected: 编译通过

- [ ] **Step 3: 提交**

```bash
git add robot/src/rt/rt_rc_interface.cpp
git commit -s -m "[New]: 手柄夺回式仲裁-网络控制期间sbus线程只监控按键

1. 网络控制期间手柄帧不覆盖rc_control
2. 任意按键按下(摇杆不参与)即夺回控制权"
```

---

### Task 5: HardwareBridge 订阅与回调

**Files:**
- Modify: `EngineAI_Controller/robot/include/HardwareBridge.h`（include 区与 `handleGamepadLCM` 声明之后）
- Modify: `EngineAI_Controller/robot/src/HardwareBridge.cpp`（`handleGamepadLCM` 函数之后、`initCommon()` 订阅处）

- [ ] **Step 1: 头文件新增 include 与回调声明**

在 `HardwareBridge.h` 的 `#include "gamepad_lcmt.hpp"` 之后追加：

```cpp
#include "rc_control_command_lcmt.hpp"
```

在 `handleGamepadLCM` 声明之后追加：

```cpp
    void handleRCControlCommandLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                   const rc_control_command_lcmt *msg);
```

- [ ] **Step 2: 源文件新增 include**

在 `HardwareBridge.cpp` 的 `#include "rt/rt_sbus.h"` 之后追加：

```cpp
#include "rt/rt_rc_interface.h"
```

- [ ] **Step 3: 源文件新增回调实现**

在 `handleGamepadLCM` 函数定义之后追加：

```cpp
/*!
 * LCM Handler for network rc command message
 */
void HardwareBridge::handleRCControlCommandLCM(const lcm::ReceiveBuffer *rbuf,
                                               const std::string &chan,
                                               const rc_control_command_lcmt *msg)
{
    (void)rbuf;
    (void)chan;
    set_rc_control_from_network(msg);
}
```

- [ ] **Step 4: 源文件注册订阅**

在 `initCommon()` 中 `_interfaceLCM.subscribe("interface_request", &HardwareBridge::handleControlParameter, this);` 之后追加：

```cpp
    _interfaceLCM.subscribe("rc_control_command", &HardwareBridge::handleRCControlCommandLCM, this);
```

- [ ] **Step 5: 编译 robot 库目标验证**

Run: `cd EngineAI_Controller/build && make robot -j$(nproc)`
Expected: 编译通过

- [ ] **Step 6: 提交**

```bash
git add robot/include/HardwareBridge.h robot/src/HardwareBridge.cpp
git commit -s -m "[New]: HardwareBridge订阅rc_control_command通道并转发写入

1. initCommon注册订阅，回调调用set_rc_control_from_network"
```

---

### Task 6: 全量编译与安装

**Files:** 无源码改动

- [ ] **Step 1: 全量编译**

Run: `cd EngineAI_Controller/build && cmake .. && make -j$(nproc)`
Expected: 编译 100% 完成，无错误（与最近提交"成功完成控制代码编译"同环境）

- [ ] **Step 2: 安装目标**

Run: `cd EngineAI_Controller/build && make install`
Expected: `EngineAI_Controller/build/EngineAI_Humanoid/` 目录生成完整安装目标

- [ ] **Step 3: 复跑单元测试确认无回归**

Run: `cd EngineAI_Controller && g++ -std=c++11 -I robot/include -I lcm-types/cpp tests/test_rt_network_command.cpp robot/src/rt/rt_network_command.cpp -o build/test_rt_network_command && ./build/test_rt_network_command`
Expected: `ALL TESTS PASSED`

---

### Task 7: 测试发送脚本

**Files:**
- Create: `EngineAI_Controller/scripts/send_rc_command.py`

- [ ] **Step 1: 创建发送脚本**

```python
#!/usr/bin/env python3
"""
向哪吒板发送 rc_control_command_lcmt 网络控制指令（等效手柄按键）。

用法（在上位机/PC 上运行，需先按 README 配置网口 LCM 组播）:
  export PYTHONPATH=<repo>/EngineAI_Controller/lcm-types/python:$PYTHONPATH
  python3 send_rc_command.py --mode 1                 # PASSIVE 开电机
  python3 send_rc_command.py --mode 2                 # STAND_UP 直腿站
  python3 send_rc_command.py --mode 11 --gait 0       # LOCOMOTION 站立(RL)
  python3 send_rc_command.py --mode 11 --gait 1 --vx 0.3   # 行走 + 前进0.3
  python3 send_rc_command.py --mode 8                 # LOCK_JOINT 保护阻尼

mode: 0=OFF 1=PASSIVE 2=STAND_UP 7=BALANCE_STAND 8=LOCK_JOINT 11=LOCOMOTION
gait: 0=站立 1=行走 (仅 LOCOMOTION 模式生效)
"""
import argparse

import lcm
from rc_control_command_lcmt import rc_control_command_lcmt


def main():
    parser = argparse.ArgumentParser(description="send rc_control_command to robot")
    parser.add_argument("--mode", type=float, required=True, help="RC_mode value")
    parser.add_argument("--gait", type=int, default=0, help="gait_type 0=stand 1=walk")
    parser.add_argument("--vx", type=float, default=0.0, help="forward velocity [-1,1]")
    parser.add_argument("--vy", type=float, default=0.0, help="lateral velocity [-1,1]")
    parser.add_argument("--wz", type=float, default=0.0, help="yaw rate [-1,1]")
    parser.add_argument("--channel", default="rc_control_command")
    args = parser.parse_args()

    msg = rc_control_command_lcmt()
    msg.mode = args.mode
    msg.gait_type = args.gait
    msg.v_des = [args.vx, args.vy, 0.0]
    msg.omega_des = [0.0, 0.0, args.wz]

    lc = lcm.LCM()
    lc.publish(args.channel, msg.encode())
    print(f"sent mode={args.mode} gait={args.gait} v=({args.vx},{args.vy}) wz={args.wz}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 语法检查**

Run: `python3 -m py_compile EngineAI_Controller/scripts/send_rc_command.py`
Expected: 无输出（通过）

- [ ] **Step 3: 提交**

```bash
git add scripts/send_rc_command.py
git commit -s -m "[New]: 新增网络控制测试发送脚本send_rc_command.py

1. 支持mode/gait/vx/vy/wz参数，经LCM通道rc_control_command发送"
```

---

### Task 8: 板载部署验证与文档

**Files:**
- Modify: `README2.md`（版本改动记录追加条目）

> 本任务需实机操作（机器人在场）。按设计文档 §5 测试计划执行。

- [ ] **Step 1: 部署到哪吒板**

按 README1 §1.3.2 流程：

```bash
# 上位机网线连接机器人，静态IP 192.168.0.100/255.255.255.0/网关192.168.0.1
sudo ifconfig <enp0s25> multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev <enp0s25>
# 拷贝安装目录并重启控制进程
scp -r build/EngineAI_Humanoid user@192.168.0.163:/home/user/
ssh user@192.168.0.163 "sudo pkill EngineAI_Controller && cd /home/user/EngineAI_Humanoid && ./run_biped.sh"
```
Expected: 板上控制进程正常启动（与手柄版部署流程相同）

- [ ] **Step 2: 安全验证（先做，人在机器人旁）**

按 ESTOP 设备使能腿电机（听到两声"Di Di"），然后：

```bash
export PYTHONPATH=<repo>/EngineAI_Controller/lcm-types/python:$PYTHONPATH
python3 send_rc_command.py --mode 11 --gait 1   # 当前OFF状态，应被拒绝
```
Expected: 板上打印 `[RC] Network command REJECTED (ret=-2, current mode=0)`，机器人保持不动

- [ ] **Step 3: 完整动作序列验证**

逐步发送并观察机器人状态（每个动作后确认无误再发下一条）：

```bash
python3 send_rc_command.py --mode 1                 # 开电机(等效LB+START)，观察电机上电
python3 send_rc_command.py --mode 2                 # 直腿站(等效LB+A)，机器人站起
python3 send_rc_command.py --mode 11 --gait 0       # 进RL站立(等效LB+X)，站立模型生效
python3 send_rc_command.py --mode 11 --gait 1       # 切行走(等效A键)，行走模型生效
python3 send_rc_command.py --mode 11 --gait 1 --vx 0.3   # 前进0.3，观察行走方向正确
python3 send_rc_command.py --mode 11 --gait 1 --vx 0.0   # 停速
python3 send_rc_command.py --mode 11 --gait 0       # 回站立(等效A键)
python3 send_rc_command.py --mode 8                 # LOCK_JOINT 保护(等效LB+RB)
```
Expected: 每次**模式变化**时板上打印 `[RC] Network command applied: mode=... gait_type=...`（同模式内更新 gait/速度静默接受、不打印，属正常设计，以机器人行为/lcm-spy 确认）；机器人动作与手柄按键逐个等效

- [ ] **Step 4: 仲裁验证**

1. 重复发送 `--mode 11 --gait 0` 保持网络控制中，按手柄任意键（如 B）→ 板上打印 `[RC] Joystick takes back control`，随后手柄恢复正常控制（再按手柄 A 应能切换步态）
2. 再发 `python3 send_rc_command.py --mode 11 --gait 0` → 网络重新接管（板上打印 applied）
Expected: 两种控制源可按上述规则双向切换，无互相覆盖抖动

- [ ] **Step 5: README2 版本记录**

在 `README2.md` 的 `### 版本改动记录` 列表末尾追加：

```markdown
3. (V-Hw01.00.00-Fw01.02.00)新增网络控制功能：外部程序经以太网LCM发送rc_control_command_lcmt消息（通道rc_control_command，字段mode/gait_type/v_des/omega_des）等效手柄按键控制机器人；手柄任意按键按下即夺回控制权；手柄原有控制逻辑不变。
   注意事项：a) 网络端按离散指令发送（每条消息均会重新接管控制权，连续流式发送会阻止手柄夺回）；b) 夺回会同时执行该按键自身动作：按A夺回会顺带切换站立/行走，按B/RB会进入校准模式——LOCOMOTION中无副作用的夺回键为单独按LB或START；c) 手柄缺失或中途拔出时无法夺回，且无网络活性超时，行走中需保证网络端或手柄至少一方可用；d) 板上applied日志仅在模式变化时打印，同模式内更新gait/速度静默接受（以机器人行为或lcm-spy确认）。
```

- [ ] **Step 6: 提交**

```bash
git add README2.md
git commit -s -m "[Modify]: README2新增网络控制功能版本记录

1. 记录网络控制消息格式、通道名与手柄夺回式仲裁"
```

---

## 备注

- **测试策略**：本仓库无测试框架，安全关键纯逻辑（mode 合法性、LOCOMOTION 前置条件、裁剪/归一化）用独立 g++ 编译的 assert 程序离线测试（Task 2）；胶水代码（互斥写入、订阅回调、仲裁）以编译验证 + 实机逐步测试覆盖（Task 6/8）。
- **部署注意**：`make_types.sh` 需在 `EngineAI_Controller/scripts/` 目录下执行；重新生成会覆盖所有既有绑定，属既有流程，无风险。
- **版本号**：README2 的 Fw01.02.00 为建议值，可按项目实际版本管理规则调整。
