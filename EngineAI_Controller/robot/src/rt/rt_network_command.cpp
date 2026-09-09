#include "rt/rt_network_command.h"

#include <cmath>

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
    // 幂等同模式更新（LOCOMOTION 内持续更新 gait_type/v_des，等价手柄摇杆连续输入）始终允许
    if (msg.mode == current_mode)
    {
        return 0;
    }
    // 仅跨模式进入 LOCOMOTION 时校验前置条件：当前必须处于 STAND_UP（与手柄 LB+X 一致）
    if (msg.mode == RC_mode::LOCOMOTION && current_mode != RC_mode::STAND_UP &&
        current_mode != RC_mode::LOCOMOTION)
    {
        return -2;
    }
    return 0;
}

void clamp_rc_network_command(rc_control_command_lcmt &msg)
{
    for (int i = 0; i < 3; i++)
    {
        // NaN 与常量的比较恒为 false，会穿透裁剪，显式置 0。
        // 注：不能用 !std::isfinite() 判断，否则 ±Inf 也会被置 0；
        // ±Inf 无需特殊处理，下方的 >/< 比较恒真会将其裁剪到 ±1。
        if (std::isnan(msg.v_des[i]))
            msg.v_des[i] = 0.0;
        else if (msg.v_des[i] > CMD_LIMIT)
            msg.v_des[i] = CMD_LIMIT;
        else if (msg.v_des[i] < -CMD_LIMIT)
            msg.v_des[i] = -CMD_LIMIT;

        if (std::isnan(msg.omega_des[i]))
            msg.omega_des[i] = 0.0;
        else if (msg.omega_des[i] > CMD_LIMIT)
            msg.omega_des[i] = CMD_LIMIT;
        else if (msg.omega_des[i] < -CMD_LIMIT)
            msg.omega_des[i] = -CMD_LIMIT;
    }

    if (msg.gait_type != 0)
    {
        msg.gait_type = 1;
    }
}
