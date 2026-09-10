# Jetson Orin：罗技手柄 → LCM 遥控发送端（开机自启）

目标：把罗技手柄插到 **Jetson Orin**，Jetson 上常驻一个转发程序，把
手柄按键/摇杆翻译成 `rc_control_command_lcmt` 消息，经以太网 LCM 发给
运动控制器（哪吒板 192.168.0.163），**板端代码零改动**，等效手柄直插板子。

本目录文件：

| 文件 | 作用 |
|---|---|
| `gamepad_rc_sender.py` | 手柄→LCM 转发主程序（复刻板端 logitech 按键逻辑） |
| `robot-rc-sender.service` | systemd 服务模板（开机自启 + 组播路由 + 崩溃自拉起） |
| `install_jetson.sh` | 一键部署脚本（装 lcm、配静态 IP、装服务） |
| `rc_control_command_lcmt.py` | 部署时从 `../EngineAI_Controller/lcm-types/python/` 自动拷贝 |

---

## 1. 架构

```
[罗技手柄] USB
   │
[Jetson Orin] gamepad_rc_sender.py (systemd 常驻)
   │ 按键组合→mode / A→gait / 摇杆→v_des,omega_des
   │ LCM udpm://239.255.76.67:7667  (网线直连)
   ▼
[哪吒板/运动控制器] 192.168.0.163
   │ 现成网络控制链路(rc_control_command 通道, Fw01.02.00)
   ▼
FSM → RL 双 ONNX → 关节 → 电机
```

按键组合与板端手柄完全一致：

| 组合 | 含义 |
|---|---|
| LB + START | PASSIVE 开电机（零力矩） |
| LB + BACK | OFF 关电机 |
| LB + RB | LOCK_JOINT 阻尼锁腿（停止时用） |
| LB + A | STAND_UP 直腿站 |
| LB + B | BALANCE_STAND 蹲站 |
| LB + X | LOCOMOTION（仅当前为 STAND_UP 时生效，与板端一致） |
| LOCOMOTION 内：A | 切换 站立模型(gait=0) / 行走模型(gait=1) |
| LOCOMOTION 内：左摇杆 | 上/下=前进/后退 vx，左/右=横移 vy |
| LOCOMOTION 内：右摇杆 | 左/右=转向 wz |

> 板端 B 键相关的“速度/姿态零偏标定”功能依赖手柄本地写入，走网络通道
> 无法标定（消息里没有 bias 字段），本程序不实现，属正常裁剪。

---

## 2. 一次性部署（需要联网）

在 **Jetson** 上（仓库已拷到 Jetson，或 `git clone`/`scp` 过去）：

```bash
cd <仓库>/jetson_rc
sudo bash install_jetson.sh eth0 192.168.0.100 192.168.0.163
```

脚本会：
1. 检查 `import lcm`，没有则 `git clone` 官方 lcm 源码编译安装（首次需联网）；
2. 拷贝程序与 `rc_control_command_lcmt.py` 到 `/opt/robot_rc/`；
3. 静态 IP `192.168.0.100/24` 配到 `eth0`（NetworkManager / netplan 自动识别，开机保持）；
4. 立即生效组播路由 `224.0.0.0/4 dev eth0`；
5. 安装并启用 `robot-rc-sender.service`（开机自启）。

网口不是 `eth0` 时先 `ip -br link` 查一下。哪吒板 IP 若不是
`192.168.0.163` 就用第 3 个参数改（板端网口必须同网段静态 IP）。

---

## 3. 验证

```bash
# 1) 服务起来了？
systemctl status robot-rc-sender
journalctl -u robot-rc-sender -f

# 2) 网络
ip addr show eth0                # 应显示 192.168.0.100/24
ip route | grep 224              # 应有 224.0.0.0/4 dev eth0
ping 192.168.0.163               # 板端可达

# 3) 手柄被识别(罗技插 USB, 建议用与直插板子相同的拨档模式)
ls -l /dev/input/js*

# 4) 按键映射核对(只打印, 不发指令到机器人也安全)
sudo python3 /opt/robot_rc/gamepad_rc_sender.py --device /dev/input/js0 --print-events
#    依次按 A/B/X/Y/LB/RB/BACK/START 与各摇杆, 应看到 -> A/B/X/Y/LB/RB/BACK/START / LX/LY/RX/RY
```

确认映射后停掉上面这个前台进程（`Ctrl+C`，它只在退出时发一次 0 速，
不在 LOCOMOTION 则不产生消息）。

---

## 4. 真机联调（人在机器人旁，ESTOP 就绪）

```bash
# 先停服务跑前台更直观(或直接看 journalctl)
sudo systemctl stop robot-rc-sender
sudo python3 /opt/robot_rc/gamepad_rc_sender.py --device /dev/input/js0
```

按 README2 里的安全序列逐步操作，每一步确认机器人动作正常再进下一步：

1. **LB+START** → 电机上电（板上日志 `applied: mode=1`）
2. **LB+A** → 直腿站起（`applied: mode=2`）
3. **LB+X** → 进入 LOCOMOTION，默认站立模型（`applied: mode=11`）
4. **A** → 切行走模型（同模式静默，机器人应有姿态变化/以 lcm-spy 确认）
5. **左摇杆上推** → 前进；**右摇杆左/右** → 转向
6. 若转向方向相反：程序加 `--invert-yaw`（改服务参数见 §6）
7. 停止：**LB+RB**（阻尼锁腿）→ 人扶稳后再 **LB+BACK**（关电机）

确认无误后：
```bash
sudo systemctl start robot-rc-sender    # 转回开机自启常驻
```

---

## 5. 开机自启服务说明

```ini
ExecStartPre=...  ip route replace 224.0.0.0/4 dev eth0   # 开机把组播路由指到机器人网口
ExecStart=python3 /opt/robot_rc/gamepad_rc_sender.py --device /dev/input/js0
Restart=always                                            # 崩溃自动拉起
```

常驻行为：
- **非 LOCOMOTION**：只在模式变化时发一条（离散，同直插手柄）；
- **LOCOMOTION**：30 Hz 周期发布摇杆量（连续，保证速度平滑）；
- 手柄中途拔出/重插：程序自动重连；拔出期间若处于 LOCOMOTION 会自动持续发
  **0 速**，避免机器人保持最后指令；
- 服务被 stop/重启（SIGTERM）：若处于 LOCOMOTION 会先发一条 0 速再退出。

管理命令：
```bash
sudo systemctl restart robot-rc-sender   # 重启
sudo systemctl stop    robot-rc-sender
sudo systemctl disable robot-rc-sender   # 取消开机自启
journalctl -u robot-rc-sender -f         # 看日志
```

---

## 6. 改运行参数（频率/死区/转向/手柄节点）

```bash
sudo systemctl edit robot-rc-sender      # 覆盖默认配置
```

填入并保存（`--rate` 发布频率、`--deadband` 摇杆死区、`--invert-yaw` 反转向、
`--device` 手柄节点）：

```ini
[Service]
ExecStart=
ExecStart=/usr/bin/python3 /opt/robot_rc/gamepad_rc_sender.py \
    --device /dev/input/js0 --rate 30 --deadband 0.02 --invert-yaw
```

```bash
sudo systemctl restart robot-rc-sender
```

---

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| `import lcm` 失败 | `sudo apt install build-essential cmake python3-dev swig git` 后重跑 `install_jetson.sh` |
| 程序报找不到 `rc_control_command_lcmt` | 确认 `/opt/robot_rc/rc_control_command_lcmt.py` 存在；该文件必须与板端 `.lcm` 同源（同 fingerprint），否则板端丢弃 |
| 板上打印 `REJECTED` | 前置条件不满足（如没先 STAND_UP 就 LB+X）；与直插手柄约束一致，按安全序列来 |
| 板上收不到消息 | `ip route|grep 224` 是否正确；板端与 Jetson 是否同网段；`journalctl -u robot-rc-sender` 是否在发 |
| `--print-events` 按键号对不上 | 手柄拨档不同（F710 的 D/X 模式）。拨到与直插板子一致的模式，或以打印出的实际序号为准在程序里改 `JS_BTN_*`/`AX_*` |
| Jetson 重启后机器人还“记得”最后指令 | 板端**无网络活性超时**。务必先停机器人再重启 Jetson；或重启后人工确认/发送安全指令 |

---

## 8. 已知限制（继承自板端网络控制设计）

- 板端没有“网络活性超时”：网络断开会保持最后一条指令。
  本程序已尽量兜底（拔手柄发 0 速、退出发 0 速），但 **Jetson 断电/网线断开瞬间
  来不及发消息**，机器人行走中请先 LB+RB/LB+A 停稳。
- 手柄缺失时无法从网络侧接管机器人：机器人在线期间保证 Jetson 与手柄至少一方可用。
- 断线期间程序仍会尝试重连手柄，恢复后按当前手柄状态继续。
