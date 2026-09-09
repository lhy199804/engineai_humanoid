#include <cstdio>
#include <limits>
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
    CHECK(validate_rc_network_command(make_msg(7.0, 0), 0.0) == 0);  // BALANCE_STAND 合法
    CHECK(validate_rc_network_command(make_msg(8.0, 0), 0.0) == 0);  // LOCK_JOINT 合法
    CHECK(validate_rc_network_command(make_msg(99.0, 0), 0.0) == -1); // 非法 mode
    CHECK(validate_rc_network_command(make_msg(3.0, 0), 0.0) == -1); // QP_STAND 非法(不在网络控制子集)
    CHECK(validate_rc_network_command(make_msg(6.0, 0), 0.0) == -1); // VISION 非法(不在网络控制子集)
    CHECK(validate_rc_network_command(make_msg(12.0, 0), 0.0) == -1); // RECOVERY_STAND 非法(不在网络控制子集)

    // ---- validate: LOCOMOTION 前置条件 ----
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 2.0) == 0);  // STAND_UP -> LOCOMOTION 允许
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 0.0) == -2); // OFF -> LOCOMOTION 拒绝
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 1.0) == -2); // PASSIVE -> LOCOMOTION 拒绝
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 7.0) == -2); // BALANCE_STAND -> LOCOMOTION 拒绝
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 8.0) == -2); // LOCK_JOINT -> LOCOMOTION 拒绝

    // ---- validate: 幂等同模式更新 ----
    CHECK(validate_rc_network_command(make_msg(11.0, 0), 11.0) == 0); // LOCOMOTION -> LOCOMOTION 允许(持续更新 gait_type/v_des)

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

    // ---- clamp: 边界值与 NaN/Inf 处理 ----
    {
        rc_control_command_lcmt msg = make_msg(11.0, 0);
        msg.v_des[0] = 1.0;  // 边界值保持不变
        msg.v_des[1] = -1.0; // 边界值保持不变
        msg.omega_des[2] = 0.9; // 界内值保持不变
        msg.v_des[2] = std::numeric_limits<double>::infinity();
        msg.omega_des[1] = std::numeric_limits<double>::quiet_NaN();
        clamp_rc_network_command(msg);
        CHECK(msg.v_des[0] == 1.0);                        // +1.0 边界保持
        CHECK(msg.v_des[1] == -1.0);                       // -1.0 边界保持
        CHECK(msg.omega_des[2] == 0.9);                    // 界内 0.9 保持
        CHECK(msg.v_des[2] == 1.0);                        // +Inf -> +1.0
        CHECK(msg.omega_des[1] == 0.0);                    // NaN -> 0.0

        msg.v_des[2] = -std::numeric_limits<double>::infinity();
        msg.omega_des[0] = std::numeric_limits<double>::quiet_NaN();
        msg.v_des[0] = std::numeric_limits<double>::quiet_NaN();
        clamp_rc_network_command(msg);
        CHECK(msg.v_des[2] == -1.0);                       // -Inf -> -1.0
        CHECK(msg.omega_des[0] == 0.0);                    // NaN -> 0.0
        CHECK(msg.v_des[0] == 0.0);                        // NaN -> 0.0

        // 双重 clamp 幂等：值不再变化
        clamp_rc_network_command(msg);
        CHECK(msg.v_des[0] == 0.0);
        CHECK(msg.v_des[1] == -1.0);
        CHECK(msg.v_des[2] == -1.0);
        CHECK(msg.omega_des[0] == 0.0);
        CHECK(msg.omega_des[1] == 0.0);
        CHECK(msg.omega_des[2] == 0.9);
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
