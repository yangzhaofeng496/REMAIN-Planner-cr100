# REMANI-Planner 性能定位记录

## 已确认的根因

1. `fsm/global_plan=true` 时，`planning_horizon` 被强制设置为 `1000`。
2. `planner_manager.cpp` 中的无条件 `|| true` 使普通重规划也重复执行联合 A*；已移除。
3. 联合初始化的主要耗时在 `SampleMani` 机械臂层级搜索、`trajShot/oneShot` 后处理，以及 mesh 碰撞检查，不在 L-BFGS。
4. `SampleMani::search()` 之前没有实际使用 `max_loop_num`；现已同时受时间和循环次数限制。

## 实测记录

| 组别 | 配置 | 总规划时间 | 关键阶段 | 结果 |
|---|---|---:|---|---|
| A | 固定姿态，原始配置 | 约 0.18 s | KinoA* 12.4 ms，SampleMani 171 ms | 成功 |
| B | global_plan=false，horizon=3，loop=500，sample=0.05 | 25.844 s | SampleMani 25.828 s；底盘 A* 15.1 ms | 成功 |
| C | B + rewire=false | 11.766 s | SampleMani 8.035 s；whole-body RRT 3.390 s | 成功 |
| D | C + 0.05 m / 0.05 rad / 5 checks | 32.111 s | 初始化 27.077 s；最终安全检查 5.034 s | 成功，随机搜索波动明显 |
| E | D + oneShot=false 诊断 | 约 8.48 s | SampleMani 8.011 s；whole-body RRT 94.7 ms | 本次失败 |

计数示例：C 组 whole-body RRT 使用 9 个节点、24 次碰撞检查、0 次 rewire；D 组 SampleMani 为 1945 个节点、5158 次碰撞检查、4305 次边插值。

## 安全性

优化后轨迹发布前保留独立高精度完整检查，按 10 ms 采样调用完整 `MMConfig::checkcollision()`。已实测通过：811 个状态全部无碰撞；该检查耗时约 5.03 s，并计入端到端总时间。

## 当前结论

关闭 rewire 带来约 54% 的有效加速，但仍无法满足 2–5 s 首次规划目标。粗化 RRT 插值的单次结果受随机搜索影响，不能仅凭一次样本认定有效。当前新的主要瓶颈是 SampleMani 的搜索和后处理，以及最终高精度安全检查；L-BFGS 仍不是瓶颈。

所有诊断参数均可通过 launch 参数覆盖，默认配置不变。成功率记录为：A 1/1、B 2/2、C 1/1、D 1/1；E 本次 0/1。D 的另一轮曾因旧路径长度越界触发 SIGSEGV，已增加边界保护。
