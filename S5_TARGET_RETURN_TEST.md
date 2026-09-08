# JAKA S5 自动 start/end 往返任务

## 动作顺序

`s5_auto_cycle` 启动后自动执行一次：

```text
确认实际位置为 end
→ 分别为 end→start 和 start→end 凑满 10 条合格轨迹
→ 执行最高分 end→start 轨迹
→ 到位检查并停留 5 秒
→ 执行最高分 start→end 轨迹
→ 到位检查后退出任务程序
```

任务结束后 `moveit_server` 继续运行，机器人保持上电使能，仅退出伺服轨迹模式。

点位单位为度：

```text
       joint_1   joint_2   joint_3   joint_4  joint_5  joint_6
start  -179.753    90.057   -90.199    90.196   91.724  -64.680
end      89.816   109.950  -132.277   201.880   94.806  -74.132
```

程序使用精确关节角，不对 joint_1 使用加减 360 度后的等效目标。

## 轨迹筛选

每个方向最多规划 30 次，直到获得 10 条合格候选。任一方向不足 10 条时，任务在运动前中止。
轨迹每段按最大单轴 1 度步长插值，并对所有检查点的末端雅可比矩阵进行 SVD：

- 最小奇异值小于 `0.01`：淘汰。
- 最大条件数大于 `200`：淘汰。
- 通过硬门槛后按以下公式评分，得分最高者执行：

```text
score = -(time_weight * duration
          + joint_travel_weight * accumulated_joint_travel
          + singularity_weight / minimum_singular_value)
```

默认权重分别为 `1.0`、`1.0`、`0.1`。阈值需要结合实际 TCP 和现场试验标定。

## 构建

仓库中的旧 `build/` 如果来自其他路径，应删除旧缓存或使用新的构建目录：

```bash
cd ~/Documents/project2026/python/jaka_s5/jaka_ros2
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to jaka_planner jaka_s5_moveit_config
source install/setup.bash
```

## 真机运行

执行前确认：机器人确为 S5；工具、负载、TCP 和 MoveIt Planning Scene 与现场一致；工作区无人；
安全区和急停有效；`jaka_driver` 等其他 SDK 客户端已停止；机器人已由人工移动至 end 点附近。

终端 1，连接 S5 并自动上电使能：

```bash
source install/setup.bash
ros2 launch jaka_planner moveit_server.launch.py ip:=<机器人IP> model:=s5
```

等待日志出现 `S5 connected, powered, enabled, and ready`。SDK 登录、上电、使能或状态检查失败时，
服务会退出且不会接受轨迹。

终端 2，启动 MoveIt 真机模式：

```bash
source install/setup.bash
ros2 launch jaka_s5_moveit_config demo.launch.py \
  use_rviz_sim:=false use_rviz:=true
```

终端 3，自动规划并执行一次往返：

```bash
source install/setup.bash
ros2 run jaka_planner s5_auto_cycle --ros-args \
  -p candidate_count:=10 \
  -p max_candidate_attempts:=30 \
  -p dwell_seconds:=5.0 \
  -p velocity_scaling:=0.02 \
  -p acceleration_scaling:=0.02
```

第三个命令没有额外确认提示。程序等待 `/moveit_server/ready` 和 MoveIt 状态后立即工作；实际位置与
end 的最大单轴误差超过 `0.5` 度时会拒绝启动。执行中取消、SDK 错误、急停、保护停机或到位超时
都会停止伺服运动并阻止后续步骤。

`moveit_server` 会先把 MoveIt 轨迹按时间重采样为 8 ms（125 Hz）关节指令。存在速度信息时使用
三次 Hermite 插值，否则使用线性插值；随后预检每个周期的关节变化不得超过 180 度/秒。执行时以
稳态时钟每 8 ms 连续调用一次 `servo_j(..., step_num=1)`。若发送周期延迟超过 8 ms 或 SDK 返回
错误，服务会立即中止轨迹并退出伺服模式。

## 主要参数

```text
candidate_count                    每个方向要求的合格轨迹数，默认 10
max_candidate_attempts             每个方向最多规划次数，默认 30
planning_time                      每次规划最大时间，默认 5 秒
dwell_seconds                      start 点停留时间，默认 5 秒
position_tolerance_degrees         start/end 到位容差，默认 0.5 度
velocity_scaling                   速度比例，默认 0.02
acceleration_scaling               加速度比例，默认 0.02
time_weight                        时间惩罚权重，默认 1.0
joint_travel_weight                累计转角惩罚权重，默认 1.0
singularity_weight                 奇异接近惩罚权重，默认 0.1
minimum_singular_value             奇异值硬门槛，默认 0.01
maximum_condition_number           条件数硬门槛，默认 200
singularity_check_step_degrees     奇异检查插值步长，默认 1 度
maximum_send_lateness              servo_j 发送最大容许延迟，默认 0.008 秒
```

真机首次运行保持 2% 比例，由操作员持续观察并手持急停。软件奇异性和碰撞检查不能替代现场风险评估。
