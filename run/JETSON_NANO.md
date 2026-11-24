# Jetson Nano 原生部署

如果旧项目已在这台机器上运行 ROS 2 Humble，直接复用原环境。脚本会检查依赖，
只编译 Livox SDK、MID-360 驱动、FAST-LIO 和 H7 MINI。默认单线程编译，
减少 FAST-LIO 编译时的内存压力。

经典 Jetson Nano 官方 JetPack 4.6.6 基于 Ubuntu 18.04。ROS 2 Humble 的官方 ARM64
二进制包面向 Ubuntu 22.04。已经有 Humble 原生环境的板子先用 `check` 检查。
还在全新 JetPack 4 上的板子，需要先准备与实际硬件型号匹配的 ROS 2 Humble 环境。
Jetson Orin Nano 可使用基于 Ubuntu 22.04 的 JetPack 6。

- [NVIDIA JetPack 4.6.6](https://developer.nvidia.com/embedded/linux-tegra-r3276)
- [ROS 2 Humble Ubuntu 安装要求](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

## 首次编译

```bash
git clone --depth 1 https://github.com/icecream127/letsgo_h7.git
cd letsgo_h7
bash run/nano.sh check
bash run/nano.sh build
```

后续在 `letsgo_h7` 目录执行 `git pull --ff-only` 和 `bash run/nano.sh build`
即可增量构建。编译报缺少依赖时按提示补到现有系统。

## 接雷达和 H743

沿用旧项目的雷达网口设置。默认配置是电脑网口 `192.168.1.50`、
MID-360 `192.168.1.111`。用 `ip -br -4 addr` 检查有线网口。若现场 IP 不同，
复制 `lidar_ws/src/ros2_to_stm32/config/mid360_real.json` 到本机配置文件，
修改其中的 `host_net_info` 和雷达地址，启动时传入
`user_config_path:=/绝对路径/mid360_real.json`。

新 H743 固件需要开启 USART3 PC Link。电脑串口路径可用
`ls -l /dev/serial/by-id/` 查看。以下用 `/dev/ttyUSB0` 作示例，按实际端口替换。
只有出现串口权限错误时，检查当前用户是否有设备权限，Ubuntu 通常通过 `dialout`
组授权，加入后重新登录生效。

先查 H743 状态。这个查询不会启动电机，运行控制节点前退出查询，避免占用同一个设备。

```bash
bash run/nano.sh status --port /dev/ttyUSB0
```

如果雷达或串口还被旧项目占用，先退出旧项目。然后前台启动整套链路。

```bash
bash run/nano.sh run serial_port:=/dev/ttyUSB0
```

此命令启动 MID-360、FAST-LIO、里程计过滤和 H7 MINI，默认关闭 RViz 和周期性整图发布。
注册点云和过滤后的里程计仍可用于诊断。进程启动后等待人工启用。

在另一个已进入项目目录的终端，确认定位正常、马头朝向目标后再启用。

```bash
bash run/nano.sh arm
```

首次启用使用当时的雷达位姿，目标在正前方 1 m。无需 A/B 踩点。
MINI 按起始直线检查航向与横向偏差，进入目标 5 cm 范围时请求收步。
`/motor_state` 的 `route` 字段显示沿线进度、剩余距离、横向偏差和航向误差，
`reason` 显示收步或拒绝启用的原因。
`bash run/nano.sh pause` 请求收步，再次启用会继续走向原目标。停稳后运行
`bash run/nano.sh reset_route`，下一次启用才会从当前位置重新计 1 m。
遥控 KEY7 执行板端停机。前台启动终端按 Ctrl+C 退出。

## 只检查定位和部署

```bash
bash run/nano.sh run --show-args
source run/environment.sh
ros2 topic hz /Odometry/verified
ros2 topic echo /motor_state --once
```

`--show-args` 只打印启动参数。后两条用于已经启动进程的状态检查。
