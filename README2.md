## engineai_humanoid - 星刃人形机器人控制代码
### 项目说明
    本项目为星刃人形机器人控制代码，整体控制框架安装众擎SA01开源代码。

### 目录

  - [目录说明](#目录说明)
  - [版本改动记录](#版本改动记录)


### 目录说明

/dep-pkgs目录为众擎SA01开源控制代码所需的电机库文件

/EngineAI_Controller目录为擎SA01开源控制代码

README1文件为众擎SA01开源控制代码说明文档

README2文件为基于众擎SA01开源控制代码，进行二次开发说明

/jetson_rc目录为上位机（Jetson Orin）手柄遥控发送端：罗技手柄插在 Jetson 上，经以太网 LCM 发送 rc_control_command_lcmt 到哪吒板，等效手柄直插板端；含一键安装脚本与 systemd 开机自启配置

### 版本改动记录

1. (V-Hw01.00.00-Fw01.00.00)修改代码及CMakeLists.txt文件，成功完成控制代码编译。
2. (V-Hw01.00.00-Fw01.01.00)修改模型加载代码，实现双模型加载（其中，站立模型为自训练模型，行走模型为众擎SA01出厂）。修改原有手柄按键机器人动作，当手柄RB+X按下时默认机器人是站立；当A按下时是机器人行走和站立切换.当前已基本实现双模型切换功能，成功完成控制代码编译。
3. (V-Hw01.00.00-Fw01.02.00)新增网络控制功能：外部程序经以太网LCM发送rc_control_command_lcmt消息（通道rc_control_command，字段mode/gait_type/v_des/omega_des）等效手柄按键控制机器人；手柄任意按键按下即夺回控制权；手柄原有控制逻辑不变。默认组播信息为LCM_DEFAULT_URL='udpm://  239.255.76.67:7667?ttl=225'。
  注意事项：a) 网络端按离散指令发送（每条消息均会重新接管控制权，连续流式发送会阻止手柄夺回）；b) 夺回会同时执行该按键自身动作：按A夺回会顺带切换站立/行走，按B/RB会进入校准模式——LOCOMOTION中无副作用的夺回键为单独按LB或START；c) 手柄缺失或中途拔出时无法夺回，且无网络活性超时，行走中需保证网络端或手柄至少一方可用；d) 板上applied日志仅在模式变化时打印，同模式内更新gait/速度静默接受（以机器人行为或lcm-spy确认）。
  【LCM测试】PC端(ubuntu 20.04)代码工程下已有基于python的测试代码，但使用lcm的python模块已装在 /usr/local/lib/python3.8/site-packages/lcm，而python3 默认搜索路径里没有这个目录（只有 dist-packages），所以直接 import lcm 会失败。
           因此需要在命令行中使用如下指令，补上 PYTHONPATH 即可
           export PYTHONPATH=/home/conner/engineai_humanoid-master/EngineAI_Controller/lcm-types/python:/usr/local/lib/python3.8/site-packages:$PYTHONPATH
           通过使用如下指令，进行验证
           python3 -c "import lcm, rc_control_command_lcmt; print('OK')"
           组播地址设置
           export LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=225'
           完整序列（人在机器人旁、ESTOP 就绪，每条确认行为正常再发下一条）：
	  命令	                                动作	                               板上日志
	--mode 1	                PASSIVE 开电机（等效 LB+START）	                 applied: mode=1
	--mode 2	                STAND_UP 站起（等效 LB+A）	                                  applied: mode=2
	--mode 11 --gait 0	        RL 站立（等效 LB+X，自训练模型）	                 applied: mode=11
	--mode 11 --gait 1	        切行走（出厂模型）	                                                   静默（同模式）
	--mode 11 --gait 1 --vx 0.3	前进 0.3	                               静默
	--mode 11 --gait 1 --vx 0	停速	                                       静默
	--mode 8	               LOCK_JOINT（等效 LB+RB）	                                 applied: mode=8
          命令发送如下示例，每条命令每条确认行为正常再发下一条
          python3 EngineAI_Controller/scripts/send_rc_command.py --mode 0

4. (V-Hw01.00.00-Fw01.02.00 / Jetson-RC01.00.00)新增上位机（Jetson Orin）手柄遥控发送端：/jetson_rc 目录（板端代码未改动，复用 Fw01.02.00 已有的网络控制链路）。
   实现方式：罗技手柄(F710/F310，与直插板端同款、同拨档模式)插在 Jetson USB，程序读取 /dev/input/jsX（js0 协议），
   复刻板端 sbus_packet_complete_logitech() 语义（按键组合选模式、A 键切 gait、摇杆→v_des/omega_des），
   经以太网 LCM 发布 rc_control_command_lcmt 到通道 rc_control_command（udpm://239.255.76.67:7667?ttl=225）。
   按键等效：LB+START=PASSIVE(1)、LB+A=STAND_UP(2)、LB+X=LOCOMOTION(11，需当前为 STAND_UP)、LB+RB=LOCK_JOINT(8)、LB+B=BALANCE_STAND(7)、LB+BACK=OFF(0)；
   LOCOMOTION 内：A 切站立(gait=0)/行走(gait=1)模型，左摇杆前后→vx、右摇杆左右→wz。
   运行策略：非 LOCOMOTION 仅在模式变化时发一条；LOCOMOTION 内按 30Hz 持续发布摇杆量；手柄拔出或服务退出时自动补发 0 速。
   部署：jetson_rc/install_jetson.sh（编译安装 lcm-python 绑定、配置静态 IP 192.168.0.100/24、安装 systemd 服务 robot-rc-sender 开机自启），用法详见 jetson_rc/README.md。
   注意事项：板端无网络活性超时，行走中禁止直接断 Jetson 电源/网线（先 LB+RB 或 LB+A 停稳）；B 键零偏标定功能未纳入网络通道。   
  






