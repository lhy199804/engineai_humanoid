#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson Orin 罗技手柄 → LCM 遥控发送端
========================================================
把插在 Jetson USB 上的罗技手柄(F710/F310，与原先直插哪吒板同款、同拨档模式)
的按键/摇杆翻译成 rc_control_command_lcmt 消息，发布到板端通道
"rc_control_command"，效果等价于“手柄直接插在哪吒板上”。

设计原则：完全复刻板上 rt_rc_interface.cpp 中 sbus_packet_complete_logitech()
的语义（按键组合选模式、A 键切步态、摇杆→v_des/omega_des），
只是把“直接写全局 rc_control”换成“经 LCM 发布到网络控制通道”。

板端按键组合（与直插手柄完全一致）：
    LB + START -> PASSIVE        (1, 开电机/零力矩)
    LB + BACK  -> OFF            (0, 关电机)
    LB + RB    -> LOCK_JOINT     (8, 阻尼锁腿, 保护)
    LB + A     -> STAND_UP       (2, 直腿站)
    LB + B     -> BALANCE_STAND  (7, 蹲站)
    LB + X     -> LOCOMOTION     (11, 仅当当前模式为 STAND_UP 时生效, 与板端一致)
    LOCOMOTION 内: A 键切 站立模型(gait=0)/行走模型(gait=1)
                  左摇杆上下=前进/后退 vx, 左右=横移 vy
                  右摇杆左右=转向 wz

依赖：仅 python3 标准库 + lcm python 绑定 + 本目录下的 rc_control_command_lcmt.py
用法：
    export PYTHONPATH=/opt/robot_rc:/usr/local/lib/python3.8/site-packages:$PYTHONPATH
    export LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=225'
    python3 /opt/robot_rc/gamepad_rc_sender.py --device /dev/input/js0
    # 联机前先核对手柄事件映射:
    python3 /opt/robot_rc/gamepad_rc_sender.py --device /dev/input/js0 --print-events
"""

import argparse
import os
import select
import signal
import struct
import sys
import time

try:
    import lcm
    from rc_control_command_lcmt import rc_control_command_lcmt
except ImportError as e:
    sys.exit("依赖缺失: %s\n请先安装 lcm python 绑定并把 rc_control_command_lcmt.py "
             "所在目录加入 PYTHONPATH（见 README.md）" % e)

# ---------------- js0 协议常量（Linux joystick API, 与板端驱动一致） ----------------
# js_event: time(u32) value(s16) type(u8) number(u8)，共 8 字节
EV_KEY = 1          # type=1: 按键
EV_ABS = 2          # type=2: 摇杆/轴

# 按键序号(js 协议 number，type=EV_KEY)：与板端 JSKEY_* 定义一致
JS_BTN_A = 0
JS_BTN_B = 1
JS_BTN_X = 2
JS_BTN_Y = 3
JS_BTN_LB = 4
JS_BTN_RB = 5
JS_BTN_BACK = 6
JS_BTN_START = 7
JS_BTN_HOME = 8

# 轴序号(js 协议 number，type=EV_ABS)：与板端 JSKEY_* 定义一致
AX_LX = 0      # 左摇杆 X -> 板端 leftStickYAnalog
AX_LY = 1      # 左摇杆 Y -> 板端 leftStickXAnalog
AX_LT = 2      # LT(未使用)
AX_RX = 3      # 右摇杆 X -> 板端 rightStickYAnalog
AX_RY = 4      # 右摇杆 Y -> 板端 rightStickXAnalog
AX_RT = 5      # RT(未使用)
AX_DPAD_X = 6  # 十字键(板端用于标定, 本程序不使用)
AX_DPAD_Y = 7

BTN_NAMES = {0: 'A', 1: 'B', 2: 'X', 3: 'Y', 4: 'LB', 5: 'RB', 6: 'BACK', 7: 'START', 8: 'HOME'}
AX_NAMES = {0: 'LX', 1: 'LY', 2: 'LT', 3: 'RX', 4: 'RY', 5: 'RT', 6: 'D-X', 7: 'D-Y'}


class RC_MODE(object):
    OFF = 0
    PASSIVE = 1
    STAND_UP = 2
    BALANCE_STAND = 7
    LOCK_JOINT = 8
    LOCOMOTION = 11


MODE_NAMES = {0: 'OFF', 1: 'PASSIVE', 2: 'STAND_UP', 7: 'BALANCE_STAND',
              8: 'LOCK_JOINT', 11: 'LOCOMOTION'}


class Gamepad(object):
    """读取 /dev/input/jsX（js 协议），维护按键/摇杆状态。"""

    def __init__(self, path):
        self.path = path
        self.fd = None
        self.buttons = {}   # number -> 0/1
        self.axes = {}      # number -> 原始值 -32767..32767

    def open(self):
        self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        # 清掉积压历史事件，避免一上来误触发
        try:
            while True:
                if not os.read(self.fd, 64):
                    break
        except (BlockingIOError, OSError):
            pass
        self.buttons = {}
        self.axes = {}
        return self.fd

    def is_open(self):
        return self.fd is not None

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def _fill(self, typ, num, val):
        if typ == EV_KEY:
            self.buttons[num] = 1 if val else 0
        elif typ == EV_ABS:
            self.axes[num] = val

    def read_events(self):
        """非阻塞读一帧所有 js_event，返回 [(type, number, value), ...]。"""
        events = []
        while True:
            try:
                chunk = os.read(self.fd, 256)
            except BlockingIOError:
                break
            except OSError:
                raise
            if not chunk:
                break
            off = 0
            while off + 8 <= len(chunk):
                t, v, typ, num = struct.unpack_from('<IhBB', chunk, off)
                off += 8
                self._fill(typ, num, v)
                events.append((typ, num, v))
        return events

    def btn(self, num):
        return 1 if self.buttons.get(num) else 0

    def axis(self, num):
        return self.axes.get(num, 0) / 32767.0  # 归一化 -1..1


class RcSender(object):
    """手柄状态 -> rc_control_command_lcmt 转发主逻辑。"""

    def __init__(self, args):
        self.args = args
        self.lc = lcm.LCM(args.lcm_url)
        self.pad = Gamepad(args.device)
        self.mode = RC_MODE.OFF          # 镜像板端 rc_control.mode 初始值(0=OFF)
        self.gait = 0
        self.prev_a = 0
        self.last_pub_mode = None
        self._last_log = 0.0
        self._print_events = args.print_events

    # ---------- 摇杆语义（复刻板端 logitech 解码: rt_rc_interface.cpp） ----------
    def sticks(self):
        """返回 (vx, vy, wz)：v_des[0] 前向, v_des[1] 横向, omega_des[2] 转向。"""
        db = self.args.deadband

        def db0(x):
            return 0.0 if abs(x) < db else x

        # 板端: v_des[0]=leftStickXAnalog=-(LY轴), v_des[1]=leftStickYAnalog=-(LX轴)
        vx = db0(-self.pad.axis(AX_LY))
        vy = db0(-self.pad.axis(AX_LX))
        # 板端: omega_des[2]=rightStickYAnalog=-(RX轴)
        wz = db0(-self.pad.axis(AX_RX))
        if self.args.invert_yaw:
            wz = -wz
        return vx, vy, wz

    # ---------- 每拍状态评估（复刻板上按键组合选模式） ----------
    def evaluate(self):
        LB = self.pad.btn(JS_BTN_LB)
        A = self.pad.btn(JS_BTN_A)
        B = self.pad.btn(JS_BTN_B)
        X = self.pad.btn(JS_BTN_X)
        RB = self.pad.btn(JS_BTN_RB)
        BACK = self.pad.btn(JS_BTN_BACK)
        START = self.pad.btn(JS_BTN_START)

        mode = self.mode  # 默认保持
        if LB and START:
            mode = RC_MODE.PASSIVE
        elif LB and BACK:
            mode = RC_MODE.OFF
        elif LB and RB:
            mode = RC_MODE.LOCK_JOINT
        elif LB and A:
            mode = RC_MODE.STAND_UP
        elif LB and B:
            mode = RC_MODE.BALANCE_STAND
        elif LB and X and self.mode == RC_MODE.STAND_UP:
            mode = RC_MODE.LOCOMOTION

        mode_changed = (mode != self.mode)
        self.mode = mode

        # A 键上升沿切步态：仅 LOCOMOTION 内生效（与板端一致）
        gait_changed = False
        if self.mode == RC_MODE.LOCOMOTION:
            if A and not self.prev_a:
                self.gait = 1 - self.gait
                gait_changed = True
            self.prev_a = A
        else:
            self.prev_a = 0

        return mode_changed, gait_changed

    def publish(self, vx=0.0, vy=0.0, wz=0.0, log=''):
        m = rc_control_command_lcmt()
        m.mode = float(self.mode)
        m.gait_type = int(self.gait)
        m.v_des = [float(vx), float(vy), 0.0]
        m.omega_des = [0.0, 0.0, float(wz)]
        self.lc.publish(self.args.channel, m.encode())
        if log:
            now = time.time()
            if now - self._last_log > 0.5:
                print('[rc] %s mode=%s gait=%d v=(%.2f,%.2f) wz=%.2f'
                      % (log, MODE_NAMES.get(self.mode, self.mode), self.gait,
                         vx, vy, wz), flush=True)
                self._last_log = now
        self.last_pub_mode = self.mode

    def safe_stop(self):
        """退出前尽力把速度清 0（若最后处于 LOCOMOTION）。"""
        if self.last_pub_mode == RC_MODE.LOCOMOTION:
            try:
                self.publish(0.0, 0.0, 0.0, log='stop')
                print('[rc] sent zero-velocity before exit', flush=True)
            except Exception as e:  # noqa
                print('[rc] safe_stop failed: %s' % e, flush=True)

    def run(self):
        interval = 1.0 / max(1.0, float(self.args.rate))
        last_pub = 0.0
        last_open_try = 0.0
        last_warn = 0.0
        stop = False

        def _sig(signum, frame):  # noqa
            nonlocal stop
            stop = True

        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        print('[rc] starting relay: dev=%s channel=%s url=%s rate=%dHz'
              % (self.args.device, self.args.channel, self.args.lcm_url,
                 int(self.args.rate)), flush=True)
        if self._print_events:
            print('[rc] --print-events: 依次动每个按键/摇杆，核对下方映射：', flush=True)

        while not stop:
            now = time.time()

            # ---------- 设备连接管理（缺失时按键/摇杆视为零） ----------
            if not self.pad.is_open():
                if now - last_open_try >= 1.0:
                    last_open_try = now
                    try:
                        self.pad.open()
                        print('[rc] gamepad connected: %s' % self.args.device, flush=True)
                    except OSError as e:
                        if now - last_warn > 5.0:
                            print('[rc] waiting gamepad %s ... (%s)' % (self.args.device, e), flush=True)
                            last_warn = now
            else:
                timeout = interval if self.mode == RC_MODE.LOCOMOTION else 0.2
                try:
                    r, _, _ = select.select([self.pad.fd], [], [], timeout)
                except OSError:
                    print('[rc] gamepad disconnected, will retry...', flush=True)
                    self.pad.close()
                else:
                    if r:
                        try:
                            events = self.pad.read_events()
                        except OSError:
                            print('[rc] gamepad read error, reopen...', flush=True)
                            self.pad.close()
                        else:
                            if self._print_events:
                                for typ, num, val in events:
                                    name = (BTN_NAMES.get(num) if typ == EV_KEY
                                            else AX_NAMES.get(num) if typ == EV_ABS
                                            else str(num))
                                    print('  type=%s num=%d val=%+6d  -> %s'
                                          % (typ, num, val, name), flush=True)

            mode_changed, gait_changed = self.evaluate()

            # ---------- 发送策略 ----------
            # LOCOMOTION: 周期性持续发送(摇杆连续 + 网络常驻)；其它模式: 仅变化时发送
            now = time.time()
            if self.mode == RC_MODE.LOCOMOTION:
                if now - last_pub >= interval:
                    if self.pad.is_open():
                        vx, vy, wz = self.sticks()
                        self.publish(vx, vy, wz)
                    else:
                        # 手柄掉了: 持续发 0 速, 避免机器人保持最后指令
                        self.publish(0.0, 0.0, 0.0, log='no-pad(zero)')
                    last_pub = now
            else:
                if mode_changed:
                    self.publish(log='mode')
                elif gait_changed:
                    self.publish(log='gait')

            # 避免手柄未插入/事件稀少时空转烧 CPU；LOCOMOTION 内 30Hz 由
            # select 超时 + 上述 now-last_pub>=interval 保证节奏
            time.sleep(0.005)

        # ---------- 退出 ----------
        self.safe_stop()
        self.pad.close()
        print('[rc] relay stopped', flush=True)


def main():
    default_url = os.environ.get('LCM_DEFAULT_URL', 'udpm://239.255.76.67:7667?ttl=225')
    ap = argparse.ArgumentParser(description='Logitech gamepad -> LCM rc_control_command relay')
    ap.add_argument('--device', default='/dev/input/js0', help='joystick device node')
    ap.add_argument('--channel', default='rc_control_command')
    ap.add_argument('--lcm-url', default=default_url, help='LCM multicast url')
    ap.add_argument('--rate', type=float, default=30.0, help='LOCOMOTION 内发布频率(Hz)')
    ap.add_argument('--deadband', type=float, default=0.02, help='摇杆死区(0~1)')
    ap.add_argument('--invert-yaw', action='store_true', help='反转转向方向')
    ap.add_argument('--print-events', action='store_true', help='打印手柄原始事件, 用于核对映射')
    args = ap.parse_args()

    try:
        RcSender(args).run()
    except KeyboardInterrupt:
        print('\n[rc] interrupted', flush=True)


if __name__ == '__main__':
    main()
