# REMAIN-Planner-cr100

ROS Noetic mobile-manipulator planner for an IR100 mobile base with a CR10 arm.

当前版本包含：

- 底盘与机械臂联合规划；
- 基于 URDF collision mesh 的环境碰撞、机械臂自碰撞和机械臂-底盘碰撞检查；
- RViz 2D Nav Goal 测试入口；
- 规划耗时、机械臂 RRT 和碰撞检查耗时日志；
- 横梁障碍物仿真场景。

## 一键运行

主机需要 Ubuntu 20.04、Docker、ROS Noetic 兼容的 X11 显示环境，并先启动 ROS master：

```bash
roscore
```

首次运行（构建镜像并启动仿真）：

```bash
cd ~/tongji/REMANI-Planner
docker build -f docker/Dockerfile -t remani-planner:noetic .
DISPLAY=:0 REMANI_BUILD=1 ./docker/run_ir100_cr10_manual.sh
```

如果镜像已经构建过，后续直接运行：

```bash
cd ~/tongji/REMANI-Planner
DISPLAY=:0 REMANI_BUILD=0 ./docker/run_ir100_cr10_manual.sh
```

脚本会启动 RViz、IR100/CR10 模型、横梁地图、碰撞检测和规划器。打开 RViz 后使用 **2D Nav Goal** 发送目标点。

如果主机不是 `:0` 显示器，将 `DISPLAY=:0` 改成实际值，例如 `DISPLAY=:1001`。

## 查看规划日志

日志中常见结果：

```text
A* initialization status=2   # 找到轨迹
reach goal                    # 轨迹执行完成
NO_PATH                      # 规划失败
collision type=1              # 机械臂与环境碰撞
collision type=2              # 机械臂与底盘碰撞
collision type=3              # 机械臂自碰撞
```

规划耗时日志包括底盘 Kino-A*、机械臂联合搜索、机械臂 RRT、碰撞采样和轨迹优化阶段。

## 原始 README

原项目说明已保留在 [`README.original.md`](README.original.md) 中。
