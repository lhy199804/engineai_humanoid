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

注意:
  - 从其他模式进入 LOCOMOTION 需当前已处于 STAND_UP（等效手柄 LB+X 前置条件）；已处于 LOCOMOTION 时重复发 mode 11 用于更新 gait/速度，始终接受。否则板端拒绝（板上打印 REJECTED 日志）
  - gait/v_des/omega_des 仅在 LOCOMOTION 模式下生效
  - 本脚本只负责发布消息，板端是否接受需看板上日志（模式变化时打印 applied，拒绝时打印 REJECTED）
"""
import argparse

import lcm
from rc_control_command_lcmt import rc_control_command_lcmt

MODE_TABLE = """\
mode 取值（RC_mode 子集）:
  0=OFF  1=PASSIVE  2=STAND_UP  7=BALANCE_STAND  8=LOCK_JOINT  11=LOCOMOTION
gait: 0=站立 1=行走 (仅 LOCOMOTION 模式生效)"""


def main():
    parser = argparse.ArgumentParser(
        description="send rc_control_command to robot",
        epilog=MODE_TABLE,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", type=int, required=True, help="RC_mode value")
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
    print(f"published mode={args.mode} gait={args.gait} v=({args.vx},{args.vy}) wz={args.wz}"
          " (check board console for applied/REJECTED)")


if __name__ == "__main__":
    main()
