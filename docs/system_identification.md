# System identification — RRBot (C3)

依据《title_ RRBot 关节.docx》第 5–6 周。目标：回答“模型错在哪里”，而不是把
一切误差归因于 MPC 参数。

## 方法（Sim2Sim，无真机）

辨识会话使用 **MPC 作执行器 + 多频正弦参考**（`sysid_reference.py`，joint 各自
0.4/0.9/1.6 Hz 谐波叠加，幅值 ≤0.47 rad）激励；输入力矩 τ 取自
`/mpc_controller/diagnostics`（布局 u 在 idx 17/18），输出 q/q̇ 取自 `/joint_states`，
按时间戳最近邻对齐后做**逐关节离散 ARX 拟合**（train/val 时间切分 70/30）：

```
q[k+1]  = q[k] + Ts·qd[k]
qd[k+1] = a·qd[k] + b·τ[k-d] + c        →  I = Ts/b,  D = (1-a)/b,  τ0 = c/b
```

延迟 d 在 0..4 内扫描取 VAF 最高者。纯拟合核在 `system_identification/fit_joint_model.py`
（numpy only，带合成真值单测），rosbag 读入在 `scripts/fit_rrbot_model.py`。

## 结果（2026-09-03，`records/sysid_rrbot_ref_20260903_234511/`）

| joint | d | I (kg·m²) | D | τ0 (Nm) | VAF(val) |
|---|---|---|---|---|---|
| joint1 | 2 | 1.478 | 0.366 | -0.030 | 0.997 |
| joint2 | 0 | 0.100 | 0.176 | +0.065 | 0.998 |

- 与 MPC 标称单位质量双积分器（I=1, D=0）相比：**joint1 有效惯量约 1.48、joint2 约
  0.10，均带阻尼**；joint1 存在约 2 步（20 ms）命令延迟。该失配量级即残差 RL /
  模型补偿（C4/C5）的目标。
- 一阶单步 VAF ≈0.997/0.998（验证集时间切分）。逐关节解耦模型不建模 joint1↔joint2
  耦合力矩；τ0 项吸收静态/重力偏置（小）。这些近似在报告中明示。

## 诚实边界与下一步
- 这是 **Gazebo Sim2Sim 模型辨识**，ground-truth 来自 URDF/Gazebo 物理，但本文
  只报等效辨识参数，不声称真机。
- 下一步（协议化）：① 5 类可解释扰动（惯量/负载、阻尼、命令延迟、噪声、丢帧）
  各 ≥5-run，逐类报告辨识可复现性与置信区间；② 独立验证记录（不只时间切分）；
  ③ 把辨识参数注入 MPC 比较标称/辨识/随机化参数表现；④ 失败分类（模型误差 vs
  测量 vs 延迟 vs 求解器 vs 控制权）。

## 复现
```bash
# 1) 单元测试（合成真值）
python3 -m pytest system_identification/tests -q
# 2) 现场辨识（WSL2，rrbot_mpc launch + sysid_reference + bag）
ros2 launch mpc_controller rrbot_mpc.launch.py
ros2 bag record -o sysid_bag /joint_states /mpc_controller/diagnostics
python3 system_identification/scripts/sysid_reference.py   # ~27 s
# 3) 拟合
python3 system_identification/scripts/fit_rrbot_model.py --bag sysid_bag
```
