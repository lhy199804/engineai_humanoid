//
// Created for network rc control (2026/09)
//

#ifndef ZQ_HUMANOID_RT_NETWORK_COMMAND_H
#define ZQ_HUMANOID_RT_NETWORK_COMMAND_H

#include "rc_control_command_lcmt.hpp"

/**
 * 校验网络控制指令合法性（纯函数，可离线测试）
 * @return 0 合法(含 LOCOMOTION 同模式持续更新); -1 mode 非法;
 *         -2 跨模式进入 LOCOMOTION 被阻止(当前非 STAND_UP 且非 LOCOMOTION)
 */
int validate_rc_network_command(const rc_control_command_lcmt &msg, double current_mode);

/**
 * v_des/omega_des 各分量裁剪到 [-1,1]（NaN 置 0，±Inf 裁剪到 ±1）；gait_type 归一化为 0/1（0=站立，非0=行走）
 */
void clamp_rc_network_command(rc_control_command_lcmt &msg);

#endif // ZQ_HUMANOID_RT_NETWORK_COMMAND_H
