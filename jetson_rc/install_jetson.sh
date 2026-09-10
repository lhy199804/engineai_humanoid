#!/usr/bin/env bash
# ============================================================================
# Jetson Orin 部署脚本：罗技手柄 -> LCM 遥控发送端（开机自启）
# ----------------------------------------------------------------------------
# 用法（在 Jetson 上, 仓库已拷到本地, 例如 /home/<user>/engineai_humanoid-master）:
#   cd engineai_humanoid-master/jetson_rc
#   sudo bash install_jetson.sh [网口名] [Jetson静态IP] [板端IP]
# 例:
#   sudo bash install_jetson.sh eth0 192.168.0.100 192.168.0.163
#
# 作用:
#   1. 检查/安装 lcm python 绑定(首次需要联网, 自动 git clone 官方 lcm 编译)
#   2. 拷贝发送程序与 rc_control_command_lcmt.py 到 /opt/robot_rc/
#   3. 配置静态 IP(192.168.0.100/24) 直连哪吒板(192.168.0.163), 开机保持
#   4. 安装并启用 systemd 服务 robot-rc-sender(开机自启 + 崩溃自动重启)
# ============================================================================
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "请用 sudo 运行:  sudo bash install_jetson.sh [iface] [jetson_ip] [board_ip]"
    exit 1
fi

# ---------------- 可配置参数 ----------------
IFACE="${1:-auto}"
JETSON_IP="${2:-192.168.0.100}"
BOARD_IP="${3:-192.168.0.163}"
LCM_URL='udpm://239.255.76.67:7667?ttl=225'
DEVICE='/dev/input/js0'
APPDIR='/opt/robot_rc'
SERVICE='robot-rc-sender'

# 本脚本所在目录的上级 = 仓库根(自动定位 lcm-types/python 消息绑定)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MSG_PY="${REPO_ROOT}/EngineAI_Controller/lcm-types/python/rc_control_command_lcmt.py"

PYTHON_BIN="$(command -v python3 || echo /usr/bin/python3)"

echo "==> 1/5 环境检查"
[[ -f "${MSG_PY}" ]] || { echo "找不到消息绑定文件: ${MSG_PY}"; exit 1; }
echo "    python3: ${PYTHON_BIN}"

# 自动探测网口: 优先选已有 192.168.0.x 地址的网口, 否则用第 1 个 up 的以太口
if [[ "${IFACE}" == "auto" ]]; then
    IFACE="$(ip -4 -o addr show | awk '/192\.168\.0\./{print $2; exit}')"
    if [[ -z "${IFACE}" ]]; then
        IFACE="$(ip -o link show | awk -F': ' '$2 ~ /^(eth|en)/ && $2 !~ /@/{print $2; exit}')"
    fi
fi
[[ -n "${IFACE}" ]] || { echo "自动探测网口失败, 请手动指定: sudo bash $0 eth0"; exit 1; }
echo "    目标网口: ${IFACE}"

# ---------------- 1. lcm python 绑定 ----------------
echo "==> 2/5 检查 lcm python 绑定"
PY_PATHS="$(ls -d /usr/local/lib/python3.*/site-packages \
                   /usr/lib/python3/dist-packages \
                   /usr/local/lib/python3/dist-packages 2>/dev/null | tr '\n' ':')"
if ! PYTHONPATH="${PY_PATHS}" ${PYTHON_BIN} -c "import lcm" 2>/dev/null; then
    echo "    未安装, 从源码编译(需要联网)..."
    apt-get update -y
    apt-get install -y build-essential cmake python3-dev swig git
    LCM_SRC="$(mktemp -d)"
    git clone --depth 1 https://github.com/lcm-proj/lcm "${LCM_SRC}/lcm"
    cmake -S "${LCM_SRC}/lcm" -B "${LCM_SRC}/lcm/build" -DLCM_ENABLE_JAVA=OFF -DLCM_ENABLE_LUA=OFF -DLCM_ENABLE_TESTS=OFF -DLCM_ENABLE_EXAMPLES=OFF
    cmake --build "${LCM_SRC}/lcm/build" -j"$(nproc)"
    make -C "${LCM_SRC}/lcm/build" install
    ldconfig
    rm -rf "${LCM_SRC}"
    # 装完再验证一次
    PY_PATHS="$(ls -d /usr/local/lib/python3.*/site-packages \
                       /usr/lib/python3/dist-packages \
                       /usr/local/lib/python3/dist-packages 2>/dev/null | tr '\n' ':')"
    if ! PYTHONPATH="${PY_PATHS}" ${PYTHON_BIN} -c "import lcm" 2>/dev/null; then
        echo "    警告: import lcm 仍失败, 请检查 python 版本路径; "
        echo "    可把 /usr/local/lib/python3.*/site-packages 加入 /etc/systemd/system/robot-rc-sender.service 的 PYTHONPATH"
    fi
else
    echo "    lcm 已可用"
fi

# ---------------- 2. 拷贝程序与消息绑定 ----------------
echo "==> 3/5 拷贝程序到 ${APPDIR}"
mkdir -p "${APPDIR}"
install -m 0755 "${SCRIPT_DIR}/gamepad_rc_sender.py"   "${APPDIR}/gamepad_rc_sender.py"
install -m 0644 "${MSG_PY}"                            "${APPDIR}/rc_control_command_lcmt.py"
echo "    已拷贝 gamepad_rc_sender.py 与 rc_control_command_lcmt.py"

# ---------------- 3. 静态 IP(直连哪吒板) ----------------
echo "==> 4/5 配置静态 IP ${JETSON_IP}/24 于 ${IFACE}"
if systemctl is-active --quiet NetworkManager 2>/dev/null && command -v nmcli >/dev/null; then
    # --- NetworkManager 路线(JetPack 默认) ---
    con="$(nmcli -t -f GENERAL.CONNECTION device show "${IFACE}" 2>/dev/null | cut -d: -f2- || true)"
    if [[ -z "${con}" || "${con}" == "--" ]]; then
        con="robot-rc"
        nmcli connection add type ethernet ifname "${IFACE}" con-name "${con}" \
              ipv4.method manual ipv4.addresses "${JETSON_IP}/24" >/dev/null
    else
        nmcli connection modify "${con}" ipv4.method manual ipv4.addresses "${JETSON_IP}/24" >/dev/null
    fi
    nmcli connection up "${con}" >/dev/null 2>&1 || nmcli device reapply "${IFACE}" >/dev/null 2>&1 || true
    echo "    已通过 NetworkManager 写入连接: ${con}"
elif command -v netplan >/dev/null; then
    # --- netplan 路线 ---
    cat > /etc/netplan/99-robot-rc.yaml <<EOF
network:
  version: 2
  ethernets:
    ${IFACE}:
      dhcp4: false
      addresses:
        - ${JETSON_IP}/24
EOF
    netplan apply
    echo "    已写入 /etc/netplan/99-robot-rc.yaml"
else
    echo "    未检测到 NetworkManager/netplan, 请手动配置静态 IP ${JETSON_IP}/24 于 ${IFACE}"
fi

# 立即可用: 开组播 + 组播路由(重启后由 systemd ExecStartPre 再次执行)
ip link set dev "${IFACE}" multicast on || true
ip route replace 224.0.0.0/4 dev "${IFACE}" 2>/dev/null || ip route add 224.0.0.0/4 dev "${IFACE}" || true

echo -n "    ping 哪吒板 ${BOARD_IP} ... "
if ping -c 1 -W 1 "${BOARD_IP}" >/dev/null 2>&1; then
    echo "OK"
else
    echo "不通(可忽略: 板子未开机或 IP 不同, 请改 BOARD_IP)"
fi

# ---------------- 4. systemd 开机自启 ----------------
echo "==> 5/5 安装 systemd 服务 ${SERVICE}"
sed -e "s|{{IFACE}}|${IFACE}|g" \
    -e "s|{{LCM_URL}}|${LCM_URL}|g" \
    -e "s|{{APPDIR}}|${APPDIR}|g" \
    -e "s|{{PYTHON}}|${PYTHON_BIN}|g" \
    -e "s|{{DEVICE}}|${DEVICE}|g" \
    "${SCRIPT_DIR}/robot-rc-sender.service" > "/etc/systemd/system/${SERVICE}.service"

systemctl daemon-reload
systemctl enable "${SERVICE}"
systemctl restart "${SERVICE}"

echo ""
echo "==================== 完成 ===================="
echo "服务状态:  systemctl status ${SERVICE}"
echo "实时日志:  journalctl -u ${SERVICE} -f"
echo "网络检查:  ip addr show ${IFACE} ; ip route show | grep 224"
echo ""
echo "下一歩(把罗技手柄插到 Jetson USB, 确认是 /dev/input/js0):"
echo "  ls /dev/input/js*"
echo "  sudo ${PYTHON_BIN} ${APPDIR}/gamepad_rc_sender.py --device /dev/input/js0 --print-events"
echo ""
echo "安全操作序列(机器人在旁, 先 ESTOP 就绪):"
echo "  LB+START 开电机 -> LB+A 站起 -> LB+X 进入 LOCOMOTION -> A 切站立/行走模型"
echo "  左摇杆上下前进/后退; 停止时 LB+RB(阻尼锁腿)"
echo "注意: 板端无网络活性超时, 机器人行走中请勿直接拔 Jetson 电源/网线!"
