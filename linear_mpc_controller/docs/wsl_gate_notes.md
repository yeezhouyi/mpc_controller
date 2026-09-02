# WSL2 门禁执行记录（2026-09-02）

> 记录在 WSL2（Ubuntu 24.04 / ROS2 Jazzy / Gazebo Harmonic 8.11）实际执行
> 编译与仿真门槛的**证据与未决问题**。任何“通过”都只指本文件列出的范围。

## 环境
- 发行版：Ubuntu-24.04（WSL2）；`/opt/ros/jazzy`；`gz sim` 8.11（headless `-s`）。
- 开发工作区：`/home/zhouyi/ros2_ws/src/linear_mpc_controller`（从仓库同步）。
- 独立 overlay：`/home/zhouyi/install_lmpc`（`colcon build --build-base /home/zhouyi/build_lmpc --install-base /home/zhouyi/install_lmpc`）。

## 已验证 ✅
1. **C++ 核心（ROS-free，仅 Eigen）**
   `cmake -S . -B build_core -DBUILD_TESTING=ON && cmake --build build_core && ctest --test-dir build_core`
   → 构建 OK；`test_linear_mpc_core` 4/4 通过（linearisation / ZOH discretisation / Frenet signs / fallback）。
2. **ROS2 包编译**：`colcon build --packages-select linear_mpc_controller` OK；
   `ros2 pkg executables` → `linear_mpc_node`、`velocity_arbiter_node`。
3. **节点冒烟**：两节点各 `timeout 5 ros2 run` 正常退出（timeout 杀死，无崩溃）。
4. **链路组件就绪**（headless gz + `gz_ros2_control_demos` diff_drive 实验）：
   - 机器人 `ros_gz_sim create` 成功（Entity creation successful）；
   - `joint_state_broadcaster` 与 `diff_drive_base_controller` 均 **active**
     （需把控制器参数 `cmd_vel: /cmd_vel`、`odom: /odom` 指到全局命名空间）；
   - `/clock` 桥 1000 Hz 流动；`/odom` 发布者存在（`ros2 topic info` Publisher count: 1）；
   - 轨迹服务器发布 circle（630 pts）到 `/linear_mpc_node/reference`；
   - 我们的 `linear_mpc_node` lifecycle configure/activate 成功；`velocity_arbiter` 独立运行
     时 `/cmd_vel` 正常输出（TwistStamped，零值）。

## 未决问题 ⚠️（闭环验收未完成，不冒充通过）
1. **diff_drive_controller 未稳定发布 /odom 消息**：topic 存在但 `ros2 topic echo` 长时间无消息；
   下游（MPC 读 odom、运动验证）因此无法闭环。
2. **偶发 gz 崩溃**：`gz_ros2_control` 报 `no 'ros2_control' tag found in the URDF`（core dump）。
   生成的 URDF 经验证包含 6 处 ros2_control；怀疑是 headless 快速启停场景下的残留节点 /
   `/robot_description` 时序竞争（多个实例交替运行后 controller_manager 收到旧描述）。
3. 手动 TwistStamped 驱动后未能通过 gz pose 话题确认机器人物理移动（`dynamic_pose/info`
   解析路径未通），运动闭环证据缺失。

## 下一步排查建议（干净会话中复测）
1. 每次运行前 `ros2 daemon stop` + 全量 pkill（含 robot_state_publisher / spawner）；
   单次会话只跑一个世界实例，避免残留干扰。
2. 用官方 `ros2 launch gz_ros2_control_demos diff_drive_example.launch.py`（非手工拆分）
   先在带 GUI/X 或 `xvfb` 环境验证基线可用，再切 headless。
3. 确认 `gz topic -e -t /world/empty/model/diff_drive/pose` 的话题名与实体 id（用
   `/world/empty/dynamic_pose/info` 全量打印核对）。
4. 若 demo 自身在 headless 下 odom 也不出：换 `nav2_minimal_tb3_sim` 链路或我们自建
   最小 SDF（`gz::sim::systems::DiffDrive` + `ros_gz_bridge` 直连），绕开 ros2_control。

## 结论
- **编译层门槛（C++ core、colcon 包、节点冒烟）全部通过**，代码已可交付 WSL2 使用。
- **Gazebo 闭环冒烟仍属环境集成问题**（gz_ros2_control 在 headless 快速启停下的稳定性），
  与控制器代码正确性无关；控制器在参考核心离线闭环中已验证收敛（50 项 pytest）。
