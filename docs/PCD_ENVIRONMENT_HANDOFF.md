# REMANI-Planner 交接文档

## 1. 当前目标

当前版本支持使用真实机车点云替代原来的测试横杠，并通过现有 GridMap/ESDF 参与底盘和机械臂联合规划。

当前 PCD 文件为：

```text
scans_bottom_xyzRemove_filtered/scansdibu_xyzRemove_filtered_1cm.pcd
```

该文件是 binary PCD，包含 `x y z` 三个字段，约有 2,745,593 个点。

## 2. 一键启动

推荐在仓库根目录执行统一入口脚本（默认 3 cm 体素降采样）：

```bash
./run_pcd_demo.sh
```

也可以直接使用底层 Docker 脚本：

```bash
./docker/run_ir100_cr10_pcd.sh
```

两个脚本都会启动 Docker 中的：

- IR100 底盘模拟器；
- CR10 机械臂模型和 `robot_state_publisher`；
- 静态 PCD 点云发布器；
- REMANI-Planner；
- RViz。

默认不会重新编译。需要强制编译时执行：

```bash
REMANI_BUILD=1 ./docker/run_ir100_cr10_pcd.sh
```

指定其他 PCD 文件：

```bash
REMANI_PCD_FILE=/path/to/other.pcd ./docker/run_ir100_cr10_pcd.sh
```

对 274 万点的环境点云启用体素降采样（推荐 0.02–0.05）：

```bash
REMANI_VOXEL_LEAF_SIZE=0.03 ./docker/run_ir100_cr10_pcd.sh
```

额外参数会直接转发给 `roslaunch`；需要在启动前清理同名容器时：

```bash
REMANI_KILL_OLD=1 ./run_pcd_demo.sh twc_tx:=-8.0 twc_yaw:=0.3
```

`run_pcd_demo.sh` 接受的环境变量：`DISPLAY`、`REMANI_BUILD`、`REMANI_PCD_FILE`、
`REMANI_VOXEL_LEAF_SIZE`、`REMANI_KILL_OLD`、`REMANI_IMAGE`、`REMANI_CONTAINER_NAME`。

## 3. 点云坐标约定

PCD 默认按自身坐标系读取，然后由发布器应用可配置的 `T_world_cloud` 映射到 `world`：

```text
PCD 原点       = PCD 帧原点
PCD X/Y/Z 轴   = PCD 帧 X/Y/Z 轴
frame_id       = world
```

`T_world_cloud` 由 `~T_world_cloud/translation`（米）和 `~T_world_cloud/rpy`（弧度，`R = Rz*Ry*Rx`）给出，默认单位变换。PCD 文件本身不会被修改。

`exp0_ir100_cr10_pcd.launch` 默认使用一组使机车落入 `20 x 8 x 3` 地图的变换（`tx=-7.9, ty=7.05, tz=0.5`），把机车放到机器人前方约 3 米处；请按真实安装调整。

## 4. 关键文件

### 启动文件

- `docker/run_ir100_cr10_pcd.sh`：Docker 一键启动入口；
- `remani_planner/plan_manage/launch/exp0_ir100_cr10_pcd.launch`：联合启动文件；
- `remani_planner/plan_manage/launch/static_pcd_world.launch`：仅启动静态 PCD 发布器。

### 点云发布器

```text
remani_planner/plan_manage/scripts/static_pcd_publisher.py
```

功能：

- 读取 ASCII 或 binary PCD；
- 检查 `x/y/z` 字段；
- 可选体素降采样 `~voxel_leaf_size`（默认 `0.0`，即关闭；按占用体素取质心）；
- 可选刚体变换 `~T_world_cloud`（默认单位变换）；
- 发布 `sensor_msgs/PointCloud2`；
- 默认发布到 `/map_generator/global_cloud`；
- 默认坐标系为 `world`；
- 使用 latched publisher，并周期性发布。

### 规划输入链路

```text
PCD
  -> /map_generator/global_cloud
  -> GridMap::cloudCallback（世界坐标全局地图）
  -> occupancy / ESDF
  -> Kino-A* / SampleMani / RRT / trajectory optimizer
  -> IsTrajSafe()
```

> 重要：静态 PCD 必须走 `/map_generator/global_cloud`。若改发到
> `/pcl_render_node/cloud`，`GridMap::cloudOdomCallback` 会把这些点当作机体坐标系
> 的深度/点云量测，再用相机位姿做一次变换，导致点云无法作为世界障碍物使用。
> `exp0.rviz` 的 `PointCloud2` 显示已经指向 `/map_generator/global_cloud`。

## 5. 验证命令

查看点云发布者和订阅者：

```bash
rostopic info /map_generator/global_cloud
```

查看坐标系：

```bash
rostopic echo -n 1 /map_generator/global_cloud/header
```

预期结果包含：

```text
frame_id: "world"
```

检查启动节点：

```bash
rosnode list
```

应包含：

```text
/static_pcd_publisher
/remani_planner_node
/fake_mm
/model_vis
/robot_state_publisher
/rviz
```

## 6. 已完成的碰撞相关修复

机械臂和底盘的 URDF collision mesh 使用统一的 URDF FK 链进行变换，避免旧的 `T_q_0_ * T_joint` 与 `robot_state_publisher` 坐标不一致。

碰撞类型定义为：

```text
0：底盘与环境
1：机械臂与环境
2：机械臂与底盘
3：机械臂自身碰撞
```

另外修复了 URDF 网格采样碰撞检查在 `safe=false` 时阈值为零的问题。现在使用由网格采样分辨率产生的接触容差；该容差不是全局安全距离，最终高精度碰撞检查仍然保留。

诊断开关位于：

```yaml
mm:
  collision_diagnostics: false
```

临时打开后可观察 `CollisionDiag` 和 `CollisionStats` 日志。

## 7. 常见问题

### RViz 能看到点云，但规划器不避障

依次检查：

```bash
rostopic info /map_generator/global_cloud
rostopic echo -n 1 /map_generator/global_cloud/header
```

确认规划器是 subscriber，且 `frame_id` 为 `world`。最常见的原因是点云发到了
`/pcl_render_node/cloud`：该话题由 `GridMap::cloudOdomCallback`（机体坐标系深度/点云融合）
消费，而不是世界坐标全局地图，因此规划器不会把它当作静态障碍物。必须使用
`/map_generator/global_cloud`。

### 点云和机器人位置偏移

确认 RViz 的 `Fixed Frame=world`，然后用 `~T_world_cloud/translation` 和
`~T_world_cloud/rpy` 调整；不要修改 PCD 文件。`exp0_ir100_cr10_pcd.launch` 已给出一组
把机车放进 `20 x 8 x 3` 地图的默认变换，可按真实安装覆盖。

### 启动时报节点重名

不要同时运行旧的 `run_ir100_cr10_manual.sh`、`run_ir100_cr10_empty_map.sh` 和新的 PCD 启动脚本。先关闭旧的 roslaunch，再启动新的脚本。

### 启动很慢或内存较高

该 PCD 约 274 万点。可设置 `~voxel_leaf_size`（或环境变量
`REMANI_VOXEL_LEAF_SIZE`）做体素降采样，例如 2–5 cm；降采样只用于环境点云，
不应替代机械臂和底盘的 URDF collision mesh。

## 8. 测试与构建记录

已验证：

- PCD 真实文件可读取，点数为 2,745,593；
- 1 cm PCD 包围盒约为 `x[9.30, 12.49] y[-10.30, -3.80] z[-0.50, 2.00]`；
- PCD 解析、体素降采样和 `T_world_cloud` 单元测试通过（9 个用例）；
- 发布器运行时验证：5 cm PCD 经 `0.1` 体素 + 平移/偏航变换后首点与离线计算一致；
- Shell 语法检查通过；
- `git diff --check` 通过；
- `roslaunch --nodes remani_planner exp0_ir100_cr10_pcd.launch` 能解析出静态点云发布器和规划节点；
- `roslaunch --dump-params` 确认 `voxel_leaf_size` 与 `T_world_cloud` 参数正确解析。

待完成（需要重启规划器后验证）：

- 用 `/map_generator/global_cloud` 路由后，确认 `grid_map/occupancy_inflate`
  的包围盒与变换后的机车点云一致；
- 端到端发送 2D Nav Goal，确认规划器绕开机车点云。
