# Code Modification Log

本文档用于记录当前 Codex 工作目录下每次代码或参数修改。代码改动、launch/config 参数调整、构建验证结果都应记录在这里，便于之后复现实验和回滚定位。

## 2026-05-06 16:39:53 CST +0800

### 修改背景

用户提供录包分析结果：真实队友附近持续有点云聚类和高反聚类，通信数据正常，但绿框停在旧位置，导致真实队友聚类落在 predict_region 外部，`CheckClusterValidation()` 无法重新关联。最终分类为：B. 有点云聚类，但聚类没有进入 predict_region。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/swarm_lio2/swarm_lio/src/MultiUAV.h`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/swarm_lio2/swarm_lio/src/MultiUAV.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/swarm_lio2/swarm_lio/src/laserMapping.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/swarm_lio2/swarm_lio/config/simulation_gpu.yaml`

### 代码修改

文件：`swarm_lio2_v1/src/swarm_lio2/swarm_lio/src/MultiUAV.h`

- 新增通信重捕获辅助函数声明 `GetCommRecenterPosition(...)`。
- 新增参数成员 `confirmed_tracker_comm_recenter_after_time` 和 `confirmed_tracker_comm_recenter_dist_thresh`。
- 修改 `CheckClusterValidation()` 接口，传入 `lidar_end_time`，用于判断 tracker 漏观测时长。

文件：`swarm_lio2_v1/src/swarm_lio2/swarm_lio/src/MultiUAV.cpp`

- 新增 `GetCommRecenterPosition()`：检查队友通信、通信时效、全局外参有效性，以及通信预测位置与当前 tracker 位置的距离。
- 修改 `ClusterExtractPredictRegion()`：默认仍以本机 confirmed tracker 位置作为 predict_region 中心；当 tracker 漏点云观测超过阈值后，允许使用通信预测位置作为重捕获搜索中心。
- 修改 `CheckClusterValidation()`：第一优先级仍按 tracker 位置关联聚类；如果失败且 tracker 已经漏观测超过阈值，则以通信预测位置再尝试关联一次。
- 通信位置只用于搜索窗重定位和聚类重关联，不直接更新 EKF，也不直接作为绿框显示位置。

文件：`swarm_lio2_v1/src/swarm_lio2/swarm_lio/src/laserMapping.cpp`

- 更新 `CheckClusterValidation()` 调用，传入当前帧 `lidar_end_time`。

### 参数修改

文件：`swarm_lio2_v1/src/swarm_lio2/swarm_lio/config/simulation_gpu.yaml`

- 新增 `confirmed_tracker_comm_recenter_after_time: 0.3`
  - confirmed tracker 漏观测超过 0.3 秒后，允许通信预测参与重捕获。
- 新增 `confirmed_tracker_comm_recenter_dist_thresh: 2.0`
  - tracker 与通信预测位置距离超过 2.0 米时拒绝重捕获，避免错误外参把搜索窗拉到远处。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

参数展开检查：

```bash
source swarm_lio2_v1/devel/setup.zsh
roslaunch swarm_lio simulation_three.launch --dump-params | rg 'confirmed_tracker_comm_recenter|actual_uav_num|valid_cluster_dist_thresh'
```

结果：

- `quad0/quad1/quad2` 均读取到：
  - `confirmed_tracker_comm_recenter_after_time: 0.3`
  - `confirmed_tracker_comm_recenter_dist_thresh: 2.0`
  - `valid_cluster_dist_thresh: 0.75`
  - `actual_uav_num: 3`

### 后续实验观察建议

- 如果绿框仍然追不上真实队友，可把 `confirmed_tracker_comm_recenter_after_time` 调低到 `0.15`。
- 如果绿框被通信重捕获拉到错误区域，可把 `confirmed_tracker_comm_recenter_dist_thresh` 从 `2.0` 降到 `1.2`。

## 2026-04-29 21:04:34 CST +0800

### 修改背景

用户反馈：bug 仍存在。刚开始静止时红框在队友身上，过几秒后消失，导致静止队友无人机进入本机建图。

进一步判断：

- 即使加入 `temp_tracker_static_grace_time`，只要队友启动阶段持续静止，超过保护时间后仍会被静态障碍物逻辑删除。
- 初始化阶段还没有确认全部队友时，临时红框不仅用于识别，也用于 `point_filter_teammate()` 剔除候选队友点云。
- 因此在“仍有未确认队友”的阶段，不应因为静止而删除临时 tracker。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

### 代码修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

- 修改 `DeleteInvalidTemporaryTracker()` 中的静态障碍物删除条件。
- 新增判断：
  - `still_finding_teammates = teammate_tracker.size() < actual_uav_num - 1`
- 只有当 `still_finding_teammates == false` 时，才允许执行静态障碍物删除逻辑。
- 也就是说：
  - 初始化阶段还有未确认队友时，静止临时 tracker 不会被静态规则删除。
  - 红框会继续存在，并继续参与点云剔除，避免静止队友无人机进入本机地图。
  - 已确认全部队友后，才清理多余静态红框。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

### 后续实验观察建议

- 启动初期队友静止时，红框应持续保留，不应几秒后消失。
- 若红框保留但仍有队友点云进图，可继续检查 `dynamic_filter_radius` 或 `point_filter_teammate()` 剔除范围。

## 2026-04-29 21:00:16 CST +0800

### 修改背景

用户反馈小 bug：刚开始时，队友身上的红框出现一下就消失。

判断原因：

- 上一轮新增了静态障碍物删除逻辑。
- 如果队友刚开始还没明显运动，临时 tracker 在短时间内位移不足，可能被误判为静态障碍物并删除。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

### 代码修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`

- 新增参数成员 `temp_tracker_static_grace_time`。

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

- 新增读取参数：
  - `multiuav/temp_tracker_static_grace_time`
  - 默认值 `5.0`。
- 修改静态障碍物删除条件：
  - 原条件：临时 tracker 存在时间超过 `temp_tracker_static_max_time` 且运动不足。
  - 新条件：还必须同时超过 `temp_tracker_static_grace_time`。
- 目的：
  - 给刚创建的队友候选红框一个保护时间，避免刚起步、刚看到队友、队友短暂停留时被误删。

### 参数修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

- `temp_tracker_static_max_time`: `3.0 -> 5.0`
  - 延后静态障碍物判定时间。
- 新增 `temp_tracker_static_grace_time: 5.0`
  - 临时 tracker 刚创建后的静态判定保护时间。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

### 后续实验观察建议

- 启动后观察队友身上的红框是否至少持续数秒，而不是一闪即消失。
- 如果障碍物红框删除太慢，可降低 `temp_tracker_static_grace_time`，但不建议低于 `3.0`。

## 2026-04-29 20:52:19 CST +0800

### 修改背景

用户反馈两个问题：

- 队友识别时间不用极致追求短，但不能太长；当前有时一直识别不到队友。
- 红框会在某个障碍物上一直寻找，这类静态障碍物不应该持续作为队友候选。
- 队友无人机仍可能在空中进入本机建图。

本次调整目标：

- 在识别精度和确认时间之间折中，避免上一轮过严导致长期识别不到。
- 为临时红框加入静态障碍物剔除和短期黑名单机制。
- 保留较大的动态队友点云剔除半径，继续抑制空中建图残留。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

### 代码修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`

- 新增 `rejected_static_objects`，用于记录被判定为静态障碍物的位置和时间。
- 新增静态障碍物相关参数：
  - `temp_tracker_static_max_time`
  - `temp_tracker_min_motion`
  - `static_reject_duration`
  - `static_reject_radius`

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

- 在构造函数中读取新增参数：
  - `multiuav/temp_tracker_static_max_time`
  - `multiuav/temp_tracker_min_motion`
  - `multiuav/static_reject_duration`
  - `multiuav/static_reject_radius`
- 修改 `DeleteInvalidTemporaryTracker()`：
  - 当临时 tracker 存在时间超过 `temp_tracker_static_max_time`，且从创建到当前的位移小于 `temp_tracker_min_motion`，则认为它更可能是静态高反障碍物。
  - 删除该临时 tracker。
  - 将其位置加入 `rejected_static_objects`。
- 修改 `CreateTempTrackerByHighIntensity()`：
  - 创建新临时 tracker 前，先清理过期静态障碍物黑名单。
  - 若候选高反点簇距离黑名单位置小于 `static_reject_radius`，则跳过，不再创建红框。
- 目的：
  - 防止红框在静态障碍物上长期停留或删除后立刻重建。

### 参数修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

- 将轨迹匹配门限从上一轮“偏严格”调整为“中等偏严格”：
  - `traj_matching_start_thresh: 0.5 -> 0.4`
  - `ave_match_error_thresh: 0.18 -> 0.22`
  - `pos_num_in_traj: 160 -> 140`
  - `min_traj_match_points: 45 -> 35`
  - `traj_match_current_dist_thresh: 0.35 -> 0.45`
  - `traj_match_max_error_thresh: 0.38 -> 0.5`
- 略放宽聚类关联距离，避免一直识别不到队友：
  - `valid_cluster_dist_thresh: 0.65 -> 0.75`
  - `valid_temp_cluster_dist_thresh: 0.75 -> 0.85`
- 新增静态障碍物剔除参数：
  - `temp_tracker_static_max_time: 3.0`
  - `temp_tracker_min_motion: 0.35`
  - `static_reject_duration: 8.0`
  - `static_reject_radius: 1.0`
- 保留：
  - `dynamic_filter_radius: 1.2`
  - 用于继续减少队友无人机在本机地图中形成空中障碍物。

### 识别队友更容易成功的情况

- 两架无人机之间存在明显相对运动，而不是到达目标点后静止。
- 运动轨迹有横向/二维变化，圆弧、斜线、绕行、前后加左右变化都比单纯直线或静止更容易匹配。
- 队友无人机在本机 LiDAR 视野内，距离不过远，且高反点数量足够。
- `ros_bridge` 通信稳定，队友 odom 时间戳与本机点云时间差较小。
- 不要在刚启动后立刻停住；建议起飞后让两机先有几秒相对运动，再发布最终目标点。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

### 后续实验观察建议

- 如果红框仍会盯住障碍物，可进一步降低 `temp_tracker_static_max_time` 到 `2.0`，或增大 `temp_tracker_min_motion` 到 `0.5`。
- 如果队友识别仍太慢，可优先小幅降低 `min_traj_match_points`，例如 `35 -> 30`。
- 如果队友空中建图仍明显，可只增大 `dynamic_filter_radius`，不要同步放宽识别门限。

## 2026-04-29 20:27:53 CST +0800

### 修改背景

用户反馈：当前效果好一些，但到达目标点后仍出现队友无人机框与队友分离。用户明确提出可以适当加强队友无人机识别要求，不必为了减少识别时间牺牲精度。

本次调整目标：

- 优先保证队友识别和外参确认正确。
- 降低低激励、少点数、局部错配情况下的误确认概率。
- 接受确认时间变长。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

### 代码修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`

- 新增成员变量 `traj_match_max_error_thresh`。

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

- 新增读取参数：
  - `multiuav/traj_match_max_error_thresh`
  - 默认值 `0.45`。
- 修改 `TrajMatching()`：
  - 原来只计算并检查平均匹配误差 `ave_match_err`。
  - 现在同时计算最大单点匹配误差 `max_match_err`。
  - 当 `max_match_err > traj_match_max_error_thresh` 时，拒绝轨迹匹配。
- 目的：
  - 避免平均误差被多数正常点掩盖，但局部轨迹错配仍通过确认。
  - 防止错误外参在无人机到达目标点、运动激励降低后表现为框体分离。

### 参数修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

- `traj_matching_start_thresh`: `0.35 -> 0.5`
  - 提高轨迹匹配启动所需激励。
- `ave_match_error_thresh`: `0.24 -> 0.18`
  - 收紧平均轨迹匹配误差门限。
- `pos_num_in_traj`: `120 -> 160`
  - 增加轨迹历史窗口，提高外参拟合可靠性。
- `min_traj_match_points`: `30 -> 45`
  - 增加轨迹匹配所需同步点数，牺牲确认速度换取精度。
- `traj_time_tolerance`: `0.05 -> 0.04`
  - 收紧时间戳配对容差，减少时间错配。
- `traj_match_current_dist_thresh`: `0.65 -> 0.35`
  - 收紧轨迹匹配通过后的当前帧位置一致性门限。
- 新增 `traj_match_max_error_thresh: 0.38`
  - 任意单个同步轨迹点误差超过该值时拒绝确认。
- `valid_cluster_dist_thresh`: `0.85 -> 0.65`
  - 收紧确认后队友聚类关联距离，减少错误点簇拉偏 tracker 的可能。
- `valid_temp_cluster_dist_thresh`: `0.95 -> 0.75`
  - 收紧临时 tracker 聚类关联距离，减少确认前错误关联。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

### 后续实验观察建议

- 预期队友确认会变慢，但框体分离问题应减少。
- 如果仍发生分离，重点记录轨迹匹配时打印的：
  - `Average Matching Error`
  - `Max Matching Error`
  - `R/t`
- 若确认太慢但稳定，可逐步只放宽 `min_traj_match_points` 或 `traj_matching_start_thresh`，不要同时放宽所有门限。

## 2026-04-29 20:19:32 CST +0800

### 修改背景

上一轮将可视化锚点改为通信预测位置后，实验效果仍不好，仍会出现队友框与真实队友无人机分离。

进一步判断原因：

- 问题更可能来自“错误轨迹匹配/错误全局外参”。
- 之前为了加快确认，降低了轨迹激励阈值、减少了最少匹配点数、放宽了平均误差门限。
- 一旦错误轨迹匹配通过并写入 `global_extrinsic_rot/trans`，后续队友框会稳定偏离真实队友。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

### 代码修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`

- 新增成员变量 `traj_match_current_dist_thresh`。

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

- 新增读取参数：
  - `multiuav/traj_match_current_dist_thresh`
  - 默认值 `0.75`。
- 在 `CreateTeammateTracker()` 中，轨迹匹配平均误差通过后，新增当前帧位置一致性校验：
  - 计算 `rot * teammate_state.pos_end + trans` 得到当前通信队友位置映射到本机世界系的位置。
  - 与临时 tracker 当前观测目标位置 `temp_tracker[index].dyn_tracker.get_state_pos()` 比较。
  - 如果距离大于 `traj_match_current_dist_thresh`，拒绝本次匹配，不写入全局外参，不创建 teammate tracker。
- 目的：
  - 防止历史轨迹平均误差偶然过关，但当前外参明显错误，导致队友框和真实无人机分离。

### 参数修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

- `traj_matching_start_thresh`: `0.22 -> 0.35`
  - 提高轨迹匹配启动门限，降低低激励轨迹下误匹配概率。
- `ave_match_error_thresh`: `0.32 -> 0.24`
  - 收紧平均轨迹匹配误差门限，优先保证确认正确。
- `pos_num_in_traj`: `90 -> 120`
  - 增加轨迹历史窗口，提高匹配稳定性。
- `min_traj_match_points`: `18 -> 30`
  - 增加最少同步轨迹点数，避免过少点数导致外参偶然拟合成功。
- `traj_time_tolerance`: `0.07 -> 0.05`
  - 收紧轨迹时间配对容差，减少时间错配。
- 新增 `traj_match_current_dist_thresh: 0.65`
  - 当前帧位置一致性门限，超过该距离则拒绝轨迹匹配结果。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

### 后续实验观察建议

- 观察是否还会出现“确认后框稳定偏离队友”的情况。
- 如果确认变慢但框不再分离，说明之前主要是误匹配导致，应在可靠基础上再逐步加快确认。
- 如果仍分离，需要记录分离时控制台是否出现 `Trajectory Matching REJECTED current position mismatch`，并检查确认成功时打印的 `R/t` 是否异常。

## 2026-04-29 20:14:49 CST +0800

### 修改背景

上一轮为了提升鲁棒性放大了预测框和关联半径，但实验中出现“队友无人机的框直接和队友无人机分离”的现象。

判断原因：

- 队友聚类关联和动态点剔除已经更多使用通信预测位置。
- 但 `VisualizeTeammateTracker()`、`VisualizeMeshUAV()`、`VisualizePredictRegion()` 仍优先使用 EKF tracker 位置绘制。
- 当 tracker 与通信预测位置短时不一致时，RViz 中会看到框与真实队友分离。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

### 代码修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

- 修改 `VisualizeTeammateTracker()`：
  - 可视化框位置优先使用通信 odom 经过 `global_extrinsic_rot/trans` 转到本机世界系后的位置。
  - 当通信位置不可用或外参未建立时，才回退到 `teammate_tracker` 的 EKF 位置。
- 修改 `VisualizeMeshUAV()`：
  - mesh 模型位置同样优先锚定到通信预测位置。
  - 修正 `has_observation` 局部变量被重复定义的问题。
- 修改 `VisualizePredictRegion()`：
  - 预测区域球体位置优先锚定到通信预测位置。
  - 使“队友框”“mesh”“预测区域”三类可视化锚点一致，避免视觉上分离。

### 参数修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

- `predict_region_radius`: `1.25 -> 1.15`
  - 略微缩小确认后的预测框，避免视觉框过大导致误以为偏离。
- `valid_cluster_dist_thresh`: `1.0 -> 0.85`
  - 收紧已确认队友的聚类关联距离，减少大框吸附到旁边错误点簇的概率。
- `valid_temp_cluster_dist_thresh`: `1.05 -> 0.95`
  - 收紧临时 tracker 的聚类关联距离，减少确认前错误关联。
- 保留 `dynamic_filter_radius: 1.2`
  - 继续使用较大的动态点剔除半径，优先避免队友进入本机地图形成空中障碍物。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

### 后续实验观察建议

- 在 RViz 中重点观察绿色/黄色队友框是否重新贴合队友无人机主体。
- 如果框仍然分离，优先检查通信外参是否已经收敛，以及 `/quadX/connected_teammate_list` 是否稳定。
- 如果框贴合但偶尔漏剔除点云，可只增大 `dynamic_filter_radius`，不要再同步增大 `valid_cluster_dist_thresh`。

## 2026-04-29 20:11:39 CST +0800

### 修改背景

针对 Marsim 双无人机仿真流程：

- 使用 `marsim` 中的 `double_drone_mid360.launch` 启动双无人机仿真。
- 使用 `swarm_lio2_v1` 中的 `simulation.launch` 完成双机定位建图与队友检测。
- 使用 `ros_bridge.launch` 代替 UDP 通信。

本次优化目标：

- 缩短无人机确认队友所需时间。
- 提高队友无人机识别和框选鲁棒性。
- 减少队友无人机在本机地图中形成空中障碍物。
- 减少短时漏检导致的黄框频繁闪烁。

### 修改文件

- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`
- `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/laserMapping.cpp`

### 参数修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/config/simulation_gpu.yaml`

- `min_high_inten_cluster_size`: `4 -> 3`
  - 降低高反射点聚类最小点数，使仿真中较稀疏的无人机高反点更容易创建临时 tracker。
- `min_cluster_size`: `6 -> 4`
  - 降低预测区域内普通聚类最小点数，减少队友点云稀疏时漏框。
- `predict_region_radius`: `1.05 -> 1.25`
  - 放大已确认队友的预测搜索区域，提高机动或估计轻微偏差时的关联成功率。
- `temp_predict_region_radius`: `0.75 -> 0.95`
  - 放大临时 tracker 搜索区域，提升确认前持续跟踪能力。
- `traj_matching_start_thresh`: `0.35 -> 0.22`
  - 降低轨迹匹配启动所需运动激励，缩短首次确认时间。
- `ave_match_error_thresh`: `0.28 -> 0.32`
  - 略微放宽轨迹匹配平均误差门限，适配仿真调度和 ros_bridge 时间抖动。
- `pos_num_in_traj`: `120 -> 90`
  - 缩短轨迹历史窗口，降低确认等待时间。
- `min_traj_match_points`: `30 -> 18`
  - 减少轨迹匹配所需同步点数，使双机仿真更快确认队友。
- `traj_time_tolerance`: `0.04 -> 0.07`
  - 放宽轨迹时间戳匹配容差，适配 ros_bridge/仿真调度抖动。
- `valid_cluster_dist_thresh`: `0.8 -> 1.0`
  - 放宽已确认队友的聚类关联距离，提高短时偏差下的识别鲁棒性。
- `valid_temp_cluster_dist_thresh`: `0.9 -> 1.05`
  - 放宽临时 tracker 的聚类关联距离，减少确认前黄框丢失。
- `same_obj_thresh`: `0.65 -> 0.8`
  - 提高同一物体判定距离，避免同一架队友无人机重复创建多个临时黄框。
- `temp_tracker_max_miss_time`: `3.0 -> 4.0`
  - 放宽临时 tracker 最大允许漏检时间，减少短时遮挡/漏检导致的删除重建。
- `dynamic_filter_radius`: `0.95 -> 1.2`
  - 放大动态队友点云剔除半径，减少队友无人机进入本机地图形成空中障碍物。
- 新增 `observation_stale_time: 1.2`
  - 短时没有互观测时不立刻显示黄框，减少可视化状态闪烁。

### 代码修改

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.h`

- 新增成员变量 `observation_stale_time`。
- 用于控制队友 tracker 可视化从正常状态变为黄色状态的过期时间。

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/MultiUAV.cpp`

- 在构造函数中新增读取参数：
  - `multiuav/observation_stale_time`
  - 默认值为 `0.8` 秒。
- 将队友 tracker 可视化黄框判定从硬编码时间改为参数控制：
  - `VisualizeTeammateTracker()`
  - `VisualizeMeshUAV()`
- 放大预测区域 radius search 的最大返回点数：
  - 临时 tracker：`150 -> 220`
  - 已确认队友 tracker：`60 -> 160`
- 清理高频调试输出：
  - 去除高反射检测每帧打印。
  - 去除聚类结果每帧打印。
  - 去除临时 tracker 检查和创建过程中的每帧打印。
  - 保留 `print_log` 控制下的必要调试信息。

文件：`swarm_lio2_v1/src/Swarm-LIO2/swarm_lio/src/laserMapping.cpp`

- 强化 `point_filter_teammate()` 中的动态队友点云剔除逻辑。
- 原逻辑只使用 `teammate_tracker` 的 EKF 位置剔除队友点。
- 新逻辑在队友通信有效且外参已建立时，同时使用：
  - 队友通过通信发送的 odom。
  - 当前估计的 `global_extrinsic_rot/trans`。
  - 本机当前位姿。
- 这样即使某一帧聚类没框住队友，也能继续利用通信预测位置从建图输入中剔除队友点云，降低空中障碍物残留概率。

### 验证结果

在目录 `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1` 下执行：

```bash
catkin_make
```

结果：

- 编译通过。
- `swarm_lio` 目标成功链接生成：
  - `/home/hr/aaworkspace/simulate_test_codex/swarm_lio2_v1/devel/lib/swarm_lio/swarm_lio`

### 后续实验观察建议

- 观察首次出现 `Trajectory Matching Find New Teammate` 或 `Found all teammates` 的时间是否缩短。
- 在 RViz 中观察队友框是否更少脱离无人机主体。
- 观察黄框是否仍频繁出现；如果仍闪烁，可继续增大 `observation_stale_time` 到 `1.5`。
- 检查本机地图中是否还残留队友无人机形成的空中点云障碍物；如果仍明显，可继续小幅增大 `dynamic_filter_radius`，例如 `1.3`。
## 2026-04-30 10:17:47 CST

### 修改目标

修复 Marsim 三机 Mid360 仿真启动文件 `triple_drone_mid360.launch`，使其能够按当前 `single_drone.xml` 的参数要求正常展开并启动三台无人机。

### 修改文件

- `marsim/src/MARSIM/test_interface/launch/triple_drone_mid360.launch`

### 修改内容

- 将 `use_gpu_` 默认值从 `false` 改为 `true`，与当前双机 Mid360 launch 和三机 custom launch 保持一致，默认使用 GPU 点云渲染节点。
- 新增三台无人机独立的初始高度参数：
  - `init_z_0`
  - `init_z_1`
  - `init_z_2`
- 新增三台无人机独立的位置指令话题参数：
  - `position_cmd_topic_0=/quad_0/planning/pos_cmd`
  - `position_cmd_topic_1=/quad_1/planning/pos_cmd`
  - `position_cmd_topic_2=/quad_2/planning/pos_cmd`
- 新增启动控制参数：
  - `wait_for_start_signal=false`
  - `start_topic=/start_figure8`
- 删除原文件中无条件启动的 `map_pub` 节点，只保留按 `use_gpu_` 条件选择的 `map_pub1/map_pub2`，避免重复发布地图。
- 将默认地图从 `small_forest01cutoff.pcd` 调整为 `trees.pcd`，与当前双机 Mid360 launch 保持一致。
- 为三个 `single_drone.xml` include 全部补齐当前必需参数：
  - `position_cmd_topic`
  - `wait_for_start_signal`
  - `start_topic`
- 第三台无人机 `drone_id=2` 现在使用：
  - `position_cmd_topic=$(arg position_cmd_topic_2)`
  - `init_z_=$(arg init_z_2)`

### 验证结果

在 `/home/hr/aaworkspace/simulate_test_codex` 下执行：

```bash
xmllint --noout marsim/src/MARSIM/test_interface/launch/triple_drone_mid360.launch
```

结果：

- XML 语法检查通过。

继续执行：

```bash
source marsim/devel/setup.zsh && roslaunch test_interface triple_drone_mid360.launch --nodes
```

结果：

- launch 能正常展开。
- 展开的三机节点包括：
  - `/quad0_quadrotor_dynamics`
  - `/quad0_cascadePID_node`
  - `/quad0_odom_visualization`
  - `/quad0_pcl_render_node`
  - `/quad1_quadrotor_dynamics`
  - `/quad1_cascadePID_node`
  - `/quad1_odom_visualization`
  - `/quad1_pcl_render_node`
  - `/quad2_quadrotor_dynamics`
  - `/quad2_cascadePID_node`
  - `/quad2_odom_visualization`
  - `/quad2_pcl_render_node`
  - `/map_pub1`
  - `/rvizvisualisation`
