# MINI 单点路线实机测试

新增地图：`lidar_ws/src/ros2_to_stm32/ros2_to_stm32/map_mini.json`。
新增任务：同目录 `tasktodo_mini.py`。

## 启动

1. 关闭原 `info_trans` 路线控制器（其进程内也有 `task_to_do`）。不能两个控制器同时发命令。
2. 复用现有雷达、FAST-LIO、pos_trans、serial_node；按原流程完成 A/B 踩点、初始化及遥控自动。
3. 执行 `bash ~/project/letsgo/run/tasktodo_mini.sh`。不需要重新编译。

条件满足后自动运行，雷达到达赛道坐标 (1, 0) 时请求站立。
`yaw_end` 为 `null`，到点时保持当前朝向。
保持原来的 5cm 距离门限与 8度前进角度门限，以及状态回传、心跳和定位超时条件。

## 坐标注意

走点和 UI 调试话题使用雷达在 A/B 赛道坐标系中的位置。雷达从 A 点 (0, 0) 沿赛道 +X 方向到 (1, 0)，目标距离为 1 m。

手动暂停仍用原 pause.sh（请求站立，不是独立急停）。结束后重新启动 mini 会从第一点开始。
未启动实机运动测试。
