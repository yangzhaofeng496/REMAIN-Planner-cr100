# IR100 全身 IK 实施与空白地图验证

## 方案依据

本任务严格依据以下两个项目实施：

- [MoveIt Mobile Base + Arm Tutorial](https://moveit.picknik.ai/humble/doc/examples/mobile_base_arm/mobile_base_arm_tutorial.html)：负责 MoveIt 中的移动底盘、机械臂、SRDF 分组、planar joint 及差速模型配置。
- [PickNik Stretch Kinematics Plugin](https://github.com/PickNikRobotics/stretch_moveit_plugins/tree/main/stretch_kinematics_plugin)：负责全身 IK 求解，必须复用其 `searchPositionIK()` 的交替求解结构。

不要把这两个链接仅作为背景参考，也不要用随机 RRT 或另一个全局优化器替代它们。若 ROS/MoveIt 版本不兼容，只做最小 API 适配。

## 目标

基于 `/Users/yangzhaofeng/Downloads/ir100_robot_bundle/` 中的 IR100 Xacro，实现“给定末端 XYZ（可选姿态）→ 6 轴机械臂关节 + 差速底盘状态”的最小可运行全身 IK，并在空白地图中验证一个目标点。

不要先改 REMAIN-Planner 的随机 RRT；先严格按上述 MoveIt 教程和 Stretch 插件做独立、可复现的验证。

## 已确认模型

入口：

```text
ir100_description/urdf/ir100_robot.xacro
```

运动链：

```text
base_link
└── arm_world_link       fixed, xyz=(0,0,0.735)
    └── arm_base_link    fixed, yaw≈1.5707
        └── joint1~joint6
            └── Link6
                └── arm_gripper_link  fixed
```

机械臂关节必须为 `joint1` 到 `joint6`，末端必须为 `arm_gripper_link`。

原始模型的左右轮关节是 fixed，不能直接当作可驱动轮。因此严格复制 Stretch 的做法：在 SRDF 中增加 `odom` 到 `base_link` 的虚拟 `planar joint`，不修改原始轮关节，不连接真实底盘控制器。

Stretch 插件是 ROS 2 / MoveIt 2 的 `ament_cmake` pluginlib 插件，接口继承 `kinematics::KinematicsBase`。其 README 和源码要求全身组正好包含两个子组：一个单自由度机械臂关节链、一个只含一个 `PLANAR` joint 的底盘组。IR100 满足机械臂链条件，但原始 URDF 不含底盘 planar joint，因此必须通过 SRDF 虚拟关节补齐。执行前必须确认当前环境是 ROS 2；如果当前 REMAIN-Planner 只有 ROS 1 环境，不能直接复制该插件运行。

## 实施步骤

### 1. 验证 Xacro

使用实际路径，不使用 `/home/hyf/...`：

```bash
xacro /Users/yangzhaofeng/Downloads/ir100_robot_bundle/ir100_description/urdf/ir100_robot.xacro > /tmp/ir100_robot.urdf
check_urdf /tmp/ir100_robot.urdf
```

若 package URI 无法解析，修复 ROS workspace/source；不要批量替换为绝对路径。

### 2. 创建 MoveIt 配置（对应 MoveIt 教程）

创建最小配置包，例如 `ir100_moveit_config`，包含：

```xml
<group name="ir100_arm">
  <chain base_link="arm_base_link" tip="arm_gripper_link" />
</group>

<group name="mobile_base">
  <joint name="base_planar_joint" />
</group>

<group name="ir100_mobile_manipulator">
  <group name="ir100_arm" />
  <group name="mobile_base" />
</group>
```

底盘 planar joint 设置为差速模型：

```xml
<joint_property joint_name="base_planar_joint"
                 property_name="motion_model"
                 value="diff_drive" />
```

同时在 SRDF 中声明教程要求的虚拟底盘关节：

```xml
<virtual_joint name="base_planar_joint"
               type="planar"
               parent_frame="odom"
               child_link="base_link" />
```

`base_planar_joint` 必须被 MoveIt 识别为 `PLANAR` 类型，不能只是普通 URDF 中的占位名称。

`kinematics.yaml` 中全身组必须使用：

```yaml
ir100_mobile_manipulator:
  kinematics_solver: stretch_kinematics_plugin/StretchKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1
```

机械臂子组可以使用 KDL 或 TRAC-IK；但全身组不能直接使用 KDL/TRAC-IK，必须由 `StretchKinematicsPlugin` 接管。应将 Stretch 插件源码放进同一个 ROS 2 工作空间编译，不要重写其算法：

```bash
cd <ros2_ws>/src
git clone https://github.com/PickNikRobotics/stretch_moveit_plugins.git
```

### 3. 按 StretchKinematicsPlugin 源码实现交替式全身 IK

完整状态：

```text
q = [joint1...joint6, base_x, base_y, base_yaw]
```

输入：当前机械臂关节、当前底盘状态、目标末端 XYZ/可选姿态。

#### Step A：插件初始化检查

必须复现 Stretch 插件的初始化逻辑：全身组必须正好包含两个子组，分别是机械臂单自由度关节链和只含一个 `PLANAR` joint 的移动底盘组。根据 MoveIt `JointModelGroup` 的实际变量顺序确定底盘变量索引，不要硬编码索引。

#### Step B：固定底盘，求机械臂

固定当前 `base_x/base_y/base_yaw`，使用 MoveIt `RobotState` 更新 link transform，再将目标末端位姿转换到机械臂 IK solver 的 base frame，使用机械臂 IK：

```text
q_arm = IK(target_pose_in_arm_base, seed=current_q_arm)
```

不要用手工 XYZ 相减替代 frame transform。Stretch 插件实际是先固定移动底盘，再把全身目标变换到机械臂子组的 base frame。

检查关节限位、末端误差和碰撞。成功则返回。

#### Step C：机械臂失败时反推底盘

固定当前机械臂姿态，用 MoveIt `RobotState` 的正运动学得到 `T_base_to_ee(q_arm)`，计算：

```text
T_world_to_base_target
  = T_world_to_ee_target * inverse(T_base_to_ee(q_arm))
```

只提取底盘平面变量：

```text
base_x_target
base_y_target
base_yaw_target
```

禁止底盘产生 z、roll、pitch。

反推旋转必须近似纯 yaw；如果包含明显 roll/pitch，判定该底盘解无效。这对应 Stretch 插件源码中的纯 yaw 检查。

#### Step D：更新底盘后重求机械臂

把底盘更新为反推结果，再次调用机械臂 IK；最多交替 10 次，使用同一个总超时并返回明确失败原因。Stretch 插件的 `getPositionIK()` 是单次尝试，`searchPositionIK()` 才执行带超时的搜索。

默认参数：

```text
position_tolerance: 0.005 m
orientation_tolerance: 0.05 rad
max_alternating_iterations: 10
ik_timeout: 0.1 s
```

这不是全空间随机 RRT，而是 Stretch 插件的交替求解：

```text
固定底盘求机械臂 → 失败后固定机械臂求底盘 → 重复
```

### 4. 按 MoveIt 教程配置差速底盘

除 `motion_model=diff_drive` 外，补充教程中的最小平移距离参数：

```xml
<joint_property joint_name="base_planar_joint"
                 property_name="min_translational_distance"
                 value="0.01" />
```

规划或后处理必须满足：

```text
dx/dt  = v*cos(yaw)
dy/dt  = v*sin(yaw)
dyaw/dt = omega
```

禁止底盘从一个位置直接横向瞬移到另一个位置。MoveIt 教程中的 planar joint 是规划模型，不是真实轮关节；真实执行仍需另行将底盘轨迹转换为 `v`、`omega`。

### 5. 编写最小空白地图测试

测试节点不得连接真实机器人、不得加载机车点云，只加载机器人和空白地面。测试调用优先使用 MoveIt 的 `setPoseTarget()` 和 `plan()`，确保 `StretchKinematicsPlugin` 通过标准 MoveIt 接口实际被调用。

不得只测试一个目标，也不得在目标失败后自动换成附近目标并把原失败隐藏。测试节点必须依次验证下面 8 个固定目标，坐标统一使用 `odom`/世界坐标，初始底盘状态为 `(0, 0, 0)`：

| 编号 | 目标 XYZ（m） | 用途 |
|---|---|---|
| T1 | `(0.35, 0.00, 1.10)` | 机械臂正前方、中等高度 |
| T2 | `(0.45, 0.25, 0.95)` | 左侧、中等高度 |
| T3 | `(0.45, -0.25, 0.95)` | 右侧、中等高度 |
| T4 | `(0.25, 0.00, 1.25)` | 较高位置 |
| T5 | `(0.35, 0.00, 0.85)` | 较低位置 |
| T6 | `(0.75, 0.00, 0.90)` | 机械臂工作空间边缘，检验底盘补偿 |
| T7 | `(0.90, 0.20, 0.80)` | 前方左侧，检验底盘与机械臂联合解 |
| T8 | `(0.90, -0.20, 0.80)` | 前方右侧，检验底盘与机械臂联合解 |

这些点是第一组验证点，不代表最终车底检测点；实际运行时必须把表中的原始目标逐点记录，即使某个点失败也不能改写其坐标。

由于 Stretch 插件的接口输入是 `geometry_msgs/Pose` 而不是裸 XYZ，所有点先使用同一个明确的传感器朝向生成 Pose。若当前验证只关心位置，则姿态误差只记录、不作为第一版成功判据；位置仍必须满足 5 mm 阈值。

测试必须：

1. 加载 IR100 Xacro/URDF；
2. 加载 MoveIt RobotModel；
3. 设置当前关节 seed 和底盘状态；
4. 对 T1 至 T8 逐点调用 `ir100_mobile_manipulator` 的全身 IK；
5. 每个目标输出 6 个机械臂关节解及底盘 x/y/yaw；
6. 用正运动学复算每个目标的末端 XYZ；
7. 对每个目标输出位置误差、姿态误差、IK 耗时和是否使用底盘补偿；
8. 在 RViz 同时显示 8 个目标点，并逐点显示规划结果；
9. 每个目标单独标记 `PASS` 或 `FAIL`，测试失败返回非零退出码；
10. 汇总 `passed/8`，不能只输出最后一个目标的结果。

必须实际运行测试，不能只做编译或静态检查。

## 完成判据

```text
Xacro 展开和 URDF 检查成功
MoveIt 能加载模型
全身组包含 6 个机械臂变量和 3 个底盘 planar 变量
8 个目标逐点完成测试并输出 PASS/FAIL
每个 PASS 目标的正运动学复算末端 XYZ 误差 < 5 mm
空白地图规划成功
RViz 显示目标与规划结果
底盘轨迹没有横向瞬移
记录实际测试命令、ROS/MoveIt 版本、耗时和误差
```

必须明确：空白地图验证只证明模型、IK 和规划链路可行，不证明真实 IR100 底盘或机械臂已经可以执行。

## 后续接入车底检测

每个检测点使用：

```text
T_i = [x_i, y_i, z_i, optional_orientation]
```

并令：

```text
seed_i = solution_(i-1)
```

这样相邻拍摄点优先保持机械臂姿态连续；只有 IK 失败时才调用现有随机 RRT 作为局部补救。
